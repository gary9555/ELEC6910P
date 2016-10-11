function [F, M] = controller(t, s, s_des)

% 1-x
% 2-y
% 3-z
% 4-xdot
% 5-ydot
% 6-zdot
% 7-10 quat
% 11-p
% 12-q
% 13-r

global params
persistent   prev_des   prev_t

if isempty(prev_t)
    delta_t = inf;
    prev_des = zeros(13,1);
else
    delta_t = t - prev_t;
end

    
m = params.mass;
g = params.grav;
I = params.I;
%kf = params.kforce;

% PID params
Kp_xy = 3;            Kd_xy = 2; 
Kp_z = 12;             Kd_z = 12;
Kp_phi_theta = 1500;     Kd_phi_theta = 15;
Kp_psi = 25;           Kd_psi = 15;
% Kp_phi_theta = 6000;     Kd_phi_theta = 60;
% Kp_psi = 100;           Kd_psi = 60;


% loop
cur_rot = QuatToRot(s(7:10));
des_rot = QuatToRot(s_des(7:10));
prev_rot = QuatToRot(prev_des(7:10));
[phi, theta, psi] = RotToRPY_ZXY(cur_rot);
[~, ~, d_psi] = RotToRPY_ZXY(des_rot);
%[~, ~, p_psi] = RotToRPY_ZXY(prev_rot);
    
acc_x_control = Kd_xy*(s_des(4)-s(4)) + Kp_xy*(s_des(1)-s(1)); %+ (s_des(4) - prev_des(4)) / delta_t;
acc_y_control = Kd_xy*(s_des(5)-s(5)) + Kp_xy*(s_des(2)-s(2)); %+ (s_des(5) - prev_des(5)) / delta_t;
acc_z_control = Kd_z *(s_des(6)-s(6)) + Kp_z *(s_des(3)-s(3)); %+ (s_des(6) - prev_des(6)) / delta_t;

phi_control = 1/g*(acc_x_control*sin(psi) - acc_y_control*cos(psi));
theta_control = 1/g*(acc_x_control*cos(psi) + acc_y_control*sin(psi));

% if d_psi>psi && s(13)<0
%     d_psi = d_psi - pi;
% elseif d_psi<psi && s(13)>0
%     d_psi = d_psi + pi;
% end

psi_control = d_psi - psi;
if psi_control>=pi
    psi_control = -(2*pi-psi_control);
elseif psi_control<=-pi
    psi_control = 2*pi+psi_control;
end


omega_dot = [Kp_phi_theta*(phi_control - phi)     + Kd_phi_theta*(s_des(11)-s(11));
             Kp_phi_theta*(theta_control - theta) + Kd_phi_theta*(s_des(12)-s(12));
             Kp_psi*(psi_control)                 + Kd_psi*(s_des(13)-s(13))      ];

F = m * (g+acc_z_control);   
M = I * omega_dot + cross(s(11:13), I*s(11:13)); 

prev_des = s_des;
prev_t = t;
%F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

end
