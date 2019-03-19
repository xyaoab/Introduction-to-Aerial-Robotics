function [F, M] = controller(t, s, s_des)
% s_des = zeros(13,1);
% [1:6]: x, y, z, xdot, ydot, zdot
% [7:10]: quaternion, qw,qx,qy,qz
% [11:13]: oemga: wx, wy, wz
global params

m = params.mass;
g = params.grav;
I = params.I;

%tuning parameters
kp_pos = [4 4 5];%[7 7 50];%[4 6 190];
kd_pos = [2.5 2.5 5];%[4 4 80];%[7 6 100];
kp_rpy = [90 90 75];%[1000 1000 100];%[70 70 20];
kd_rpy = [15 15 10];%[98 98 49];%[30 30 10];

%F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M
%position control 
Rot = quaternion_to_R(s(7:10));
[phi,theta,yawangle] = RotToRPY_ZXY(Rot);
Rot_des = quaternion_to_R(s_des(7:10));
[phi_des,theta_des,yawangle_des] = RotToRPY_ZXY(Rot_des);
%newton equation, linearization 
p_dot_dot_des = [g*(theta_des * cos(yawangle_des)+phi_des*sin(yawangle_des))...
                g*(theta_des * sin(yawangle_des)-phi_des*cos(yawangle_des))...
                0];
w_des = s_des(11:13);
w = s(11:13);
%PD control
p1_dot_dot_c = p_dot_dot_des(1) + kd_pos(1)*(s_des(4)-s(4)) + kp_pos(1)*(s_des(1)-s(1));
p2_dot_dot_c = p_dot_dot_des(2) + kd_pos(2)*(s_des(5)-s(5)) + kp_pos(2)*(s_des(2)-s(2));
p3_dot_dot_c = p_dot_dot_des(3) + kd_pos(3)*(s_des(6)-s(6)) + kp_pos(3)*(s_des(3)-s(3));
F = m*(g+p3_dot_dot_c);
phi_c = 1/g*(p1_dot_dot_c*sin(yawangle) - p2_dot_dot_c*cos(yawangle));
theta_c = 1/g*(p1_dot_dot_c*cos(yawangle) + p2_dot_dot_c*sin(yawangle));

%attitude control
yaw_angle_dif = yawangle_des-yawangle;
if yaw_angle_dif >= pi
    yaw_angle_dif=yaw_angle_dif-2*pi;
elseif yaw_angle_dif <= -pi
    yaw_angle_dif=yaw_angle_dif+2*pi;
end
rpy_dot_dot_c = [kp_rpy(1)*(phi_c-phi)+kd_rpy(1)*(0-w(1));
                 kp_rpy(2)*(theta_c-theta)+kd_rpy(2)*(0-w(2));
                 kp_rpy(3)*(yaw_angle_dif)+kd_rpy(3)*(0-w(3))]; %notsure abt derivative
M = I*rpy_dot_dot_c + cross(w, I*w);
end
