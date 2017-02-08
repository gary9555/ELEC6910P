#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);   // imu covariance matrix
MatrixXd Rt = MatrixXd::Identity(6,6);     // visual odometry covariance matrix

// state matrix initialize to all zeros
static VectorXd state = VectorXd::Zero(15);  // {position, orientation, linear velocity, gyro bias, accel bias}T
static MatrixXd covar = MatrixXd::Identity(15,15)*0.1;
static double t_prev;
static bool first = true;
#define TEST
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    /*
        header:
        uint32 seq
        time stamp
        string frame_id

        Imu:
        std_msgs/Header header
        geometry_msgs/Quaternion orientation
        float64[9] orientation_covariance
        geometry_msgs/Vector3 angular_velocity
        float64[9] angular_velocity_covariance
        geometry_msgs/Vector3 linear_acceleration
        float64[9] linear_acceleration_covariance
    */
    if(first){
        t_prev = msg->header.stamp.toSec();
        first = false;
    }else{
        double t_now = msg->header.stamp.toSec();
        double t_diff = t_now - t_prev;
        t_prev = t_now;
        //cout<<t_diff<<endl;
        
        //static double past_t = msg->header.stamp.toSec();
        //double cur_t = msg->header.stamp.toSec();
        //cout<<"past: "<<past_t;
        //cout<<"curr: "<<cur_t;  
        //double t_diff=cur_t-past_t;
        //past_t=cur_t;
        //cout<<t_diff<<endl;
        VectorXd deriv = VectorXd::Zero(15);
        deriv(0) = state(6);
        deriv(1) = state(7);
        deriv(2) = state(8);



        double phi = state(3);
        double theta = state(4);
        double psi = state(5);

        Matrix3d G;
        G<< cos(theta), 0,     -cos(phi)*sin(theta),
                0,      1,          sin(phi),
            sin(theta), 0,     cos(phi)*cos(theta);
        
        Vector3d omega, acc;
        omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        
        Vector3d x4;
        x4 << state(9), state(10), state(11);
        
        Vector3d x2 = G.inverse()*(omega-x4);
        deriv(3) = x2(0);
        deriv(4) = x2(1);
        deriv(5) = x2(2);

        Vector3d g;
        g << 0, 0, 9.8;
        Matrix3d Rot;
        Rot << cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(psi)*sin(phi),
            cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
            -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

        Vector3d x5;
        x5(0) = state(12);
        x5(1) = state(13);
        x5(2) = state(14);
        Vector3d x3 = g + Rot*(acc - x5);
        
        deriv(6) = x3(0);
        deriv(7) = x3(1);
        deriv(8) = x3(2);

        // udpate state vector
        state = state + t_diff*deriv;   // state vector update

        Matrix3d G_inv = G.inverse();
        Matrix3d dG_dphi;
        dG_dphi << 0, 0, sin(phi)*sin(theta),
                0, 0, cos(phi),
                0, 0, -sin(phi)*cos(theta);
        Matrix3d dG_dtheta;
        dG_dtheta << -sin(theta), 0, -cos(phi)*cos(theta),
                    0, 0, 0,
                    cos(theta), 0, -cos(phi)*sin(theta);
                
        // for orientation
        MatrixXd dG_dx2_phi = -G_inv*dG_dphi*G_inv*(omega-x4);
        MatrixXd dG_dx2_theta = -G_inv*dG_dtheta*G_inv*(omega-x4);
        
        
        // for linear velocity
        Matrix3d dR_dphi;
        dR_dphi << -cos(phi)*sin(psi)*sin(theta), sin(phi)*sin(psi),    cos(theta)*cos(phi)*sin(psi),
                cos(psi)*cos(phi)*sin(theta),  -sin(phi)*cos(psi),   -cos(psi)*cos(theta)*cos(phi),
                sin(phi)*sin(theta),           cos(phi),             -sin(phi)*cos(theta);
        Matrix3d dR_dtheta;
        dR_dtheta << -cos(psi)*sin(theta),  0,   cos(psi)*cos(theta)-sin(theta)*sin(phi)*sin(psi),
                    -sin(theta)*sin(psi)+cos(psi)*sin(phi)*cos(theta), 0,  sin(psi)*cos(theta)+cos(psi)*sin(theta)*sin(phi),
                    -cos(phi)*cos(theta),   0,   -cos(phi)*sin(theta);
        Matrix3d dR_dpsi;
        dR_dpsi << -sin(psi)*cos(theta)-sin(phi)*cos(psi)*sin(theta),   -cos(phi)*cos(psi),   -sin(psi)*sin(theta)+cos(theta)*sin(phi)*cos(psi),
                cos(theta)*cos(psi)-sin(psi)*sin(phi)*sin(theta),    -cos(phi)*sin(psi),   cos(psi)*sin(theta)+sin(psi)*cos(theta)*sin(phi),
                0, 0, 0;

        MatrixXd dR_dx2_phi = dR_dphi*(acc - x5);
        MatrixXd dR_dx2_theta = dR_dtheta*(acc - x5);
        MatrixXd dR_dx2_psi = dR_dpsi*(acc - x5);


        MatrixXd A_t(15,15);
        A_t <<  0, 0, 0,   0, 0, 0,    1, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 1, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 1,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   dG_dx2_phi(0), dG_dx2_theta(0), 0,  0, 0, 0,    -G_inv(0,0), -G_inv(0,1), -G_inv(0,2),   0, 0, 0,
                0, 0, 0,   dG_dx2_phi(1), dG_dx2_theta(1), 0,  0, 0, 0,    -G_inv(1,0), -G_inv(1,1), -G_inv(1,2),   0, 0, 0,
                0, 0, 0,   dG_dx2_phi(2), dG_dx2_theta(2), 0,  0, 0, 0,    -G_inv(2,0), -G_inv(2,1), -G_inv(2,2),   0, 0, 0,
                0, 0, 0,   dR_dx2_phi(0), dR_dx2_theta(0), dR_dx2_psi(0),  0, 0, 0,    0, 0, 0,        -Rot(0,0), -Rot(0,1), -Rot(0,2),
                0, 0, 0,   dR_dx2_phi(1), dR_dx2_theta(1), dR_dx2_psi(1),  0, 0, 0,    0, 0, 0,        -Rot(1,0), -Rot(1,1), -Rot(1,2),
                0, 0, 0,   dR_dx2_phi(2), dR_dx2_theta(2), dR_dx2_psi(2),  0, 0, 0,    0, 0, 0,        -Rot(2,0), -Rot(2,1), -Rot(2,2),
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0;
    
    
    
        MatrixXd F_t = MatrixXd::Identity(15,15) + t_diff*A_t;
        MatrixXd V_t(15,12);
        V_t <<  0, 0, 0,     0, 0, 0,    0, 0, 0,    0, 0, 0,  //
                0, 0, 0,     0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,     0, 0, 0,    0, 0, 0,    0, 0, 0,
                -G_inv(0,0), -G_inv(0,1), -G_inv(0,2),     0, 0, 0,    0, 0, 0,    0, 0, 0,  //
                -G_inv(1,0), -G_inv(1,1), -G_inv(1,2),     0, 0, 0,    0, 0, 0,    0, 0, 0,
                -G_inv(2,0), -G_inv(2,1), -G_inv(2,2),     0, 0, 0,    0, 0, 0,    0, 0, 0,
                0, 0, 0,     -Rot(0,0), -Rot(0,1), -Rot(0,2),    0, 0, 0,    0, 0, 0,  //
                0, 0, 0,     -Rot(1,0), -Rot(1,1), -Rot(1,2),    0, 0, 0,    0, 0, 0,
                0, 0, 0,     -Rot(2,0), -Rot(2,1), -Rot(2,2),    0, 0, 0,    0, 0, 0,
                0, 0, 0,     0, 0, 0,    1, 0, 0,    0, 0, 0,  //
                0, 0, 0,     0, 0, 0,    0, 1, 0,    0, 0, 0,
                0, 0, 0,     0, 0, 0,    0, 0, 1,    0, 0, 0,
                0, 0, 0,     0, 0, 0,    0, 0, 0,    1, 0, 0,  //
                0, 0, 0,     0, 0, 0,    0, 0, 0,    0, 1, 0,
                0, 0, 0,     0, 0, 0,    0, 0, 0,    0, 0, 1;
    
    
        V_t = V_t*t_diff;
        covar = F_t*covar*F_t.transpose()+ V_t*Q*V_t.transpose();    // covariance matrix update
    // t_prev = t_now;
    }
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;  // Ric
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
   // ROS_INFO("odom_callback");
    /*
        Odometry:
        std_msgs/Header header
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
            geometry_msgs/Pose pose
                geometry_msgs/Point position
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Quaternion orientation
                    float64 x
                    float64 y
                    float64 z
                    float64 w
            float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
            geometry_msgs/Twist twist
                geometry_msgs/Vector3 linear
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Vector3 angular
                float64 x
                float64 y
                float64 z
            float64[36] covariance

        header:
        uint32 seq
        time stamp
        string frame_id
    */


    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //RotationMatrix << -1, 0, 0,
    //	                 0, 1, 0,
    //                   0, 0, -1;	
 #ifdef TEST   
    MatrixXd C_t(6,15);
    C_t <<  1, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0, 
            0, 1, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0, 
            0, 0, 1,    0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0, 
            0, 0, 0,    1, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0, 
            0, 0, 0,    0, 1, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0, 
            0, 0, 0,    0, 0, 1,    0, 0, 0,    0, 0, 0,    0, 0, 0;
    MatrixXd W_t = MatrixXd::Identity(6,6);    
    MatrixXd temp = C_t*covar*C_t.transpose() + W_t*Rt*W_t.transpose();
    MatrixXd K_t = covar*C_t.transpose()*temp.inverse();
    
    double obs_x = msg->pose.pose.position.x;
    double obs_y = msg->pose.pose.position.y;
    double obs_z = msg->pose.pose.position.z;
    //VectorXd ori_temp = Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,  
      //                              msg->pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,0,1);
    Matrix3d Rcw_rot = Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,  \
                                    msg->pose.pose.orientation.z).toRotationMatrix();
   // if(ori_temp(0) > M_PI)
    //    ori_temp(0) -= 2*M_PI;
    //else if(ori_temp(0) < -M_PI)
     //   ori_temp(0) += 2*M_PI;

    // Rcw euler angles
  //  double ori_phi = ori_temp(1);
   // double ori_theta = ori_temp(2);
    //double ori_psi = ori_temp(0);
    /////////////////////////////  Rcw
   // Matrix3d Rcw_rot;
   // Rcw_rot <<  cos(ori_psi)*cos(ori_theta)-sin(ori_phi)*sin(ori_psi)*sin(ori_theta), -cos(ori_phi)*sin(ori_psi), cos(ori_psi)*sin(ori_theta)+cos(ori_theta)*sin(ori_phi)*sin(ori_psi),
    //      cos(ori_theta)*sin(ori_psi)+cos(ori_psi)*sin(ori_phi)*sin(ori_theta), cos(ori_phi)*cos(ori_psi),  sin(ori_psi)*sin(ori_theta)-cos(ori_psi)*cos(ori_theta)*sin(ori_phi),
      //    -cos(ori_phi)*sin(ori_theta),                             sin(ori_phi),           cos(ori_phi)*cos(ori_theta);
    MatrixXd Rcw = MatrixXd::Zero(4,4);
    Rcw.topLeftCorner(3,3) = Rcw_rot;
    Rcw(0,3) = obs_x;
    Rcw(1,3) = obs_y;
    Rcw(2,3) = obs_z;
    Rcw(3,3) = 1;
    /////////////////////////// Ric
    MatrixXd Ric = MatrixXd::Zero(4,4);
    Ric.topLeftCorner(3,3) = Rcam;
    Ric(0,3) = 0;
    Ric(1,3) = -0.04;
    Ric(2,3) = -0.02;
    Ric(3,3) = 1;

    // transform the odometry pose data to the imu frame
   // Vector4d position;
   // position << obs_x, obs_y, obs_z, 1;
   // position = Ric*position;

    MatrixXd Riw = Ric*Rcw;
    MatrixXd Rwi = Riw.inverse();
    Matrix3d ori_matrix = Rwi.topLeftCorner(3,3);
    VectorXd angle = ori_matrix.eulerAngles(2,0,1);
    // measurement update
    VectorXd z_t(6);
    z_t << Rwi(0,3), Rwi(1,3), Rwi(2,3), angle(1), angle(2), angle(0); 
    VectorXd g_t(6);
    g_t << state(0),state(1),state(2), state(3), state(4), state(5);

    VectorXd z_min_g = z_t - g_t;
    if(z_min_g(3) > M_PI)
        z_min_g(3) -= 2*M_PI;
    else if(z_min_g(3) < -M_PI)
        z_min_g(3) += 2*M_PI;

    if(z_min_g(4) > M_PI)
        z_min_g(4) -= 2*M_PI;
    else if(z_min_g(4) < -M_PI)
        z_min_g(4) += 2*M_PI;

    if(z_min_g(5) > M_PI)
        z_min_g(5) -= 2*M_PI;
    else if(z_min_g(5) < -M_PI)
        z_min_g(5) += 2*M_PI;    
    
    state = state + K_t*(z_min_g);  // update state
    covar = covar - K_t*C_t*covar;    // update covariance matrix
    
    double phi = state(3);
    double theta = state(4);
    double psi = state(5);

    Matrix3d R;
    R <<  cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
          cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi),  sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
          -cos(phi)*sin(theta),                             sin(phi),           cos(phi)*cos(theta);
    
    // transform the imu frame back to world frame
/*    MatrixXd P = MatrixXd::Zero(4,4);
    P.topLeftCorner(3,3) = R;
    P(0,3) = state(0);
    P(1,3) = state(1);
    P(2,3) = state(2);
    P(3,3) = 1;

    
    P = Riw.inverse()*P;
*/
    //Matrix3d t = P.topLeftCorner(3,3);
    Quaterniond Quat(R);
   // Quat = t;
   // Quat = P.topLeftCorner(3,3);
   
    nav_msgs::Odometry ekf_odom;
    ekf_odom.header.stamp = ros::Time::now();
    ekf_odom.header.frame_id = "world";
    ekf_odom.pose.pose.position.x = state(0);
    ekf_odom.pose.pose.position.y = state(1);
    ekf_odom.pose.pose.position.z = state(2);
    ekf_odom.pose.pose.orientation.w = Quat.w();
    ekf_odom.pose.pose.orientation.x = Quat.x();
    ekf_odom.pose.pose.orientation.y = Quat.y();
    ekf_odom.pose.pose.orientation.z = Quat.z();
    ekf_odom.twist.twist.linear.x = state(6);
    ekf_odom.twist.twist.linear.y = state(7);
    ekf_odom.twist.twist.linear.z = state(8);
    odom_pub.publish(ekf_odom);
#endif
   
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 5);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();  // spin -180 degrees along y axis 
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // initialize covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);   
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6); 
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);  
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3); 
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1); 

    ros::spin();
}
