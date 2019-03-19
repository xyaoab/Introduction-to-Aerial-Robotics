function s_des = circle_trajectory(t, true_s)
    
    s_des = zeros(13,1);

    omega1=25;
    omega2=20;
    x_des=4*cos((-t)*omega1/180*pi);
    y_des=3*sin((-t)*omega2/180*pi);
    z_des=3-3/625*t*t;          

    x_vdes=omega1/180*pi*4*sin((-t)*omega1/180*pi);   
    y_vdes= -omega2/180*pi*3*cos((-t)*omega2/180*pi);
    z_vdes=-6/625*t;           

    s_des(1)=x_des;
    s_des(2)=y_des;
    s_des(3)=z_des;
    s_des(4)=x_vdes;
    s_des(5)=y_vdes;
    s_des(6)=z_vdes;
    
    %desired yaw angle in the flight
    des_yaw = mod(0.1 * pi * t,2 * pi);
    ypr = [des_yaw, 0.0, 0.0];
    Rot = ypr_to_R(ypr);
    q_des = R_to_quaternion(Rot);
    s_des(7:10) = q_des;
end
