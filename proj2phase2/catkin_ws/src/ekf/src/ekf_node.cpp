#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd mean = MatrixXd::Zero(15,1);
MatrixXd mean_temp = MatrixXd::Zero(15,1);
MatrixXd cov = MatrixXd::Identity(15,15);
MatrixXd cov_temp = MatrixXd::Identity(15,15);
Vector3d g(0,0,9.8); //Direction

double t = 0;
bool flag = false;


void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //ROS_INFO("IMU CALLBACK");

    //your code for propagation
    double cur_t = msg->header.stamp.toSec();
    if(!flag){
      t = cur_t;
      flag = true;
      return;
    }
    double delta_t = cur_t - t;
    t = cur_t;
    //cout << "====================" << endl;
    //cout << cov << endl;
    Vector3d ang_v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d acc_raw(msg->linear_acceleration.x, msg->linear_acceleration.y,msg->linear_acceleration.z);
    //cout << "ang_v" << ang_v << endl;

    //Build up the At matrix
    MatrixXd At = MatrixXd::Zero(15,15);
    MatrixXd Ut = MatrixXd::Zero(15,12);
    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);

    double phi = mean(3,0);
    double theta = mean(4,0);
    double psi = mean(5,0);



    Vector3d y_temp = ang_v;// - mean.block<3,1>(9,0);
    Vector3d w = y_temp-mean.block<3,1>(9,0);
    Vector3d acc = acc_raw-mean.block<3,1>(12,0);
    Matrix3d G, Ginv, Rot, R_acc_dot, Ginv_w_dot;
    G <<    cos(theta),    0,     -cos(phi)*sin(theta),
            0,             1,           sin(phi),
            sin(theta),    0,      cos(phi)*cos(theta);

    Ginv<<   cos(theta),                       0,               sin(theta),
            (sin(phi)*sin(theta))/cos(phi),    1,              -(cos(theta)*sin(phi))/cos(phi),
            -sin(theta)/cos(phi),              0,               cos(theta)/cos(phi);
    Ginv_w_dot << 0,                                                                w(2)*cos(theta) - w(0)*sin(theta),                      0,
                 -(w(2)*cos(theta) - w(0)*sin(theta))/pow(cos(phi),2),              (sin(phi)*(w(0)*cos(theta) + w(2)*sin(theta)))/cos(phi), 0,
                 (sin(phi)*(w(2)*cos(theta) - w(0)*sin(theta)))/pow(cos(phi),2),  -(w(0)*cos(theta) + w(2)*sin(theta))/cos(phi),            0;
    R_acc_dot  << sin(psi)*(acc(1)*sin(phi) + acc(2)*cos(phi)*cos(theta) - acc(0)*cos(phi)*sin(theta)), acc(2)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)),  -acc(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(2)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acc(1)*cos(phi)*cos(psi),
                  -cos(psi)*(acc(1)*sin(phi) + acc(2)*cos(phi)*cos(theta) - acc(0)*cos(phi)*sin(theta)), acc(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)),   acc(0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc(1)*cos(phi)*sin(psi),
                  acc(1)*cos(phi) - acc(2)*cos(theta)*sin(phi) + acc(0)*sin(phi)*sin(theta),           -cos(phi)*(acc(0)*cos(theta) + acc(2)*sin(theta)),                                                                            0;
    At.block<3,3>(3,3) = Ginv_w_dot;
    At.block<3,3>(3,9) = -G.inverse();
    At.block<3,3>(6,3) = R_acc_dot;
    Ut.block<3,3>(3,0) = -G.inverse();

    Rot << cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
           cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
           -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

    At.block<3,3>(6,12) = -Rot;
    Ut.block<3,3>(6,3) = -Rot;
    Ut.block<6,6>(9,6) = MatrixXd::Identity(6,6);
    /*
    R_dot_entry <<  z(1)*sin(phi)*sin(psi) + z(2)*cos(phi)*cos(theta)*sin(psi) - z(0)*cos(phi)*sin(psi)*sin(theta), z(2)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - z(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), -z(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - z(2)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - z(1)*cos(phi)*cos(psi),
                    z(0)*cos(phi)*cos(psi)*sin(theta) - z(2)*cos(phi)*cos(psi)*cos(theta) - z(1)*cos(psi)*sin(phi), z(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - z(0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)),   z(0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + z(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - z(1)*cos(phi)*sin(psi),
                    z(1)*cos(phi) - z(2)*cos(theta)*sin(phi) + z(0)*sin(phi)*sin(theta),-z(0)*cos(phi)*cos(theta) - z(2)*cos(phi)*sin(theta), 0;
    */

    //Discretization

    MatrixXd Ft = MatrixXd::Identity(15,15) + delta_t*At;
    //Build up Ut matrix

    MatrixXd Vt = delta_t*Ut;
    //cov_temp = Ft*cov*Ft.transpose()+Vt*Q*Vt.transpose();
    cov = Ft*cov*(Ft.transpose())+Vt*Q*(Vt.transpose());
    //ROS_INFO("==================");
    VectorXd f_value = MatrixXd::Zero(15,1);
    f_value.block<3,1>(0,0) = mean.block<3,1>(6,0);
    f_value.block<3,1>(3,0) = Ginv*w;
    f_value.block<3,1>(6,0) = g + Rot*acc;
    //mean_temp = mean + delta_t*f_value;
    mean += delta_t*f_value;


    nav_msgs::Odometry odom;
    Matrix3d Rwm;
    Rwm << 0,1,0,
           1,0,0,
           0,0,-1;
    Vector3d w_p = Rwm*mean.block<3,1>(0,0);
    Vector3d w_v = Rwm*mean.block<3,1>(6,0);
    odom.header.frame_id = "world";
    odom.header.stamp = msg->header.stamp;
    odom.pose.pose.position.x = w_p(0,0);
    odom.pose.pose.position.y = w_p(1,0);
    odom.pose.pose.position.z = w_p(2,0);

    odom.twist.twist.linear.x = w_v(0);
    odom.twist.twist.linear.y = w_v(1);
    odom.twist.twist.linear.z = w_v(2);

    double phi_t = mean(3,0);
    double theta_t = mean(4,0);
    double psi_t = mean(5,0);
    Matrix3d R_rot;
    R_rot << cos(psi_t)*cos(theta_t)-sin(phi_t)*sin(psi_t)*sin(theta_t), -cos(phi_t)*sin(psi_t), cos(psi_t)*sin(theta_t)+cos(theta_t)*sin(phi_t)*sin(psi_t),
              cos(theta_t)*sin(psi_t)+cos(psi_t)*sin(phi_t)*sin(theta_t), cos(phi_t)*cos(psi_t), sin(psi_t)*sin(theta_t)-cos(psi_t)*cos(theta_t)*sin(phi_t),
              -cos(phi_t)*sin(theta_t), sin(phi_t), cos(phi_t)*cos(theta_t);
    R_rot = Rwm*R_rot;
    Quaterniond ori(R_rot);

    odom.pose.pose.orientation.x = ori.x();
    odom.pose.pose.orientation.y = ori.y();
    odom.pose.pose.orientation.z = ori.z();
    odom.pose.pose.orientation.w = ori.w();


    odom_pub.publish(odom);
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //					   RotationMatrix << -1, 0, 0,
    //							                  0, 1, 0,
    //                               0, 0, -1;
    //ROS_INFO("ODOM CALLBACK");

    ///
    Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    Matrix3d cam_R_w = rot.toRotationMatrix();

    Vector3d cam_T_w(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    Matrix3d R = cam_R_w.transpose() * Rcam.transpose();
    Vector3d pos_i_w = -cam_R_w.transpose() * (Rcam.transpose() * Vector3d(0,-0.04,-0.02) + cam_T_w);

    double phi = asin(R(2,1));
    double psi = atan2(-R(0,1)/cos(phi),R(1,1)/cos(phi));
    double theta = atan2(-R(2,0)/cos(phi),R(2,2)/cos(phi));


    Vector3d ori_i_w(phi, theta, psi);
    MatrixXd Ct = MatrixXd::Zero(6,15);
    Ct.block<6,6>(0,0) = MatrixXd::Identity(6,6);
    MatrixXd Wt = MatrixXd::Identity(6,6);

    MatrixXd temp = Ct*cov*(Ct.transpose())+Wt*Rt*(Wt.transpose());
    MatrixXd Kt = cov*(Ct.transpose())*(temp.inverse());
    VectorXd imu_pos(6,1);
    imu_pos.block<3,1>(3,0) = ori_i_w;
    imu_pos.block<3,1>(0,0) = pos_i_w;

    VectorXd odom_diff = imu_pos-Ct*mean;
    if(odom_diff(5) > M_PI){
      odom_diff(5) -= 2*M_PI;
    }
    else if(odom_diff(5) < -M_PI){
      odom_diff(5) += M_PI*2;
    }
    if(odom_diff(4) > M_PI){
      odom_diff(4) -= 2*M_PI;
    }
    else if(odom_diff(4) < -M_PI){
      odom_diff(4) += M_PI*2;
    }
    if(odom_diff(3) > M_PI){
      odom_diff(3) -= 2*M_PI;
    }
    else if(odom_diff(3) < -M_PI){
      odom_diff(3) += M_PI*2;
    }

    mean += Kt*odom_diff;
    cov -= Kt*Ct*cov;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;

    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
