#ifndef control_funtion_h
#define control_funtion_h

#include <math.h>

/*please fill this controller function
 * input:
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle
 * now_pos -> now psition
 * now_vel -> body velocity
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 *
 
 * output:
 * rpy -> target attitude for autopilot
 * target_thrust     -> target thrust of the quadrotor
 * */
void SO3Control_function( const double des_pos[3],
                          const double des_vel[3],
                          const double des_acc[3],
                          const double des_yaw,
                          const double now_pos[3],
                          const double now_vel[3],
                          const double now_yaw,
                          const double Kp[3],
                          const double Kd[3],
                          const double Mass,
                          const double Gravity,
                          double rpy[3],
                          double &target_thrust
                        )
{

        std::cout << "my control function " << std::endl;
	
	double acc_x_control = Kd[0]*(des_vel[0] - now_vel[0]) + Kp[0]*(des_pos[0] - now_pos[0]);
	double acc_y_control = Kd[1]*(des_vel[1] - now_vel[1]) + Kp[1]*(des_pos[1] - now_pos[1]);
	double acc_z_control = Kd[2]*(des_vel[2] - now_vel[2]) + Kp[2]*(des_pos[2] - now_pos[2]);

        rpy[0] = (acc_x_control*sin(des_yaw) - acc_y_control*cos(des_yaw)) / Gravity;
	rpy[1] = (acc_x_control*cos(des_yaw) + acc_y_control*sin(des_yaw)) / Gravity;

	double psi_control = des_yaw - now_yaw;
	if(psi_control >= M_PI)
		psi_control -=2*M_PI;
	else if(psi_control <= -M_PI)
		psi_control += 2*M_PI;
	else;
	rpy[2] = psi_control;

	target_thrust = Mass*(Gravity + acc_z_control);
	

	 
      
       //std::cout << "ROLL:   "<<rpy[0]<<std::endl;
        //std::cout<< "PITCH:   "<<rpy[1]<<std::endl;
        //std::cout << "YAW:   "<<rpy[2]<<std::endl;
        std::cout << "thrust "<<target_thrust << std::endl;
	   std::cout << "pos0:   "<<now_pos[0]<<std::endl;
	   std::cout << "pos1:   "<<now_pos[1]<<std::endl;
	std::cout << "pos2:   "<<now_pos[2]<<std::endl;

std::cout << "des0:   "<<des_pos[0]<<std::endl;
	   std::cout << "des1:   "<<des_pos[1]<<std::endl;
	std::cout << "des2:   "<<des_pos[2]<<std::endl;

	std::cout<<"Kp2  "<<Kp[2]<<std::endl;
	std::cout<<"Kd2  "<<Kd[2]<<std::endl;


}
#endif
