function s_des = trajectory_generator(t, path, h, map)

persistent ts;
persistent ts_delta;
persistent P;
persistent xyz;
if nargin > 1 % pre-process can be done here (given waypoints)
%% time for each segments 

total_time = 25.0;
path_seg_length = sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2));
ts = cumsum(path_seg_length);
ts = ts/ts(end);
ts = [0; ts]';
ts = ts*total_time;
ts_delta = ts(2:end) - ts(1:end-1); % interval 
%% cost function 
m = length(path)-1; %# segments 
Q_dia = zeros(8*m, 8*m); %block diagonal matrix
for k=1:m
    for i=4:8
        for j=4:8
            Q_dia((k-1)*8+i,(k-1)*8+j)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(j+i-7)*(ts_delta(k)^(i+j-7));
        end
    end
end

%% derivative constraint -- 0th derivative %A_j *p_j = d_j
dim = 2*m+4*(m-1)+ 3*2; %continutity[4]+starting[3]+ending[3]
P = zeros(8*m,3); %p_j_i = 8mx3 [p00, p01...;p10...]
A = zeros(dim,8*m); %dimx8m
d= zeros(dim,3);
d(1:2*m,:) =  reshape([path(1:end-1, :) path(2:end, :)]',3, [])';
 %dimx3
for j=1:m %position
    A(2*j-1:2*j,8*j-7:8*j) = [ 1, 0, 0, 0, 0, 0, 0, 0;...
        1, ts_delta(j), ts_delta(j)^2, ts_delta(j)^3, ...
        ts_delta(j)^4, ts_delta(j)^5, ts_delta(j)^6, ts_delta(j)^7];
end
%% continutity pos,vel,acce,jerk
for j=1:(m-1)
    A((2*m+4*j-3):(2*m+4*j),(8*j-7):(8*j+8))=[1, ts_delta(j), ts_delta(j)^2, ts_delta(j)^3, ...
        ts_delta(j)^4, ts_delta(j)^5, ts_delta(j)^6, ts_delta(j)^7,-1,0,0,0,0,0,0,0;...%pos
        0, 1, 2*ts_delta(j), 3*ts_delta(j)^2, 4*ts_delta(j)^3, 5*ts_delta(j)^4, ...
        6*ts_delta(j)^5, 7*ts_delta(j)^6,0,-1,0,0,0,0,0,0;...%vel
        0, 0, 2, 6*ts_delta(j), 12*ts_delta(j)^2, 20*ts_delta(j)^3, ...
        30*ts_delta(j)^4, 42*ts_delta(j)^5,0,0,-2,0,0,0,0,0;...%acc
         0, 0, 0, 6, 24*ts_delta(j), 60*ts_delta(j)^2, ...
        120*ts_delta(j)^3, 210*ts_delta(j)^4,0,0,0,-6,0,0,0,0];%jerk
end
%% starting point vel,acc, jerk 
A(6*m-3:6*m-1, 1:8)=[0,1,0,0,0,0,0,0;...
                     0,0,2,0,0,0,0,0;...
                     0,0,0,6,0,0,0,0];
A(6*m:6*m+2,8*m-7:8*m) = [0, 1, 2*ts_delta(m), 3*ts_delta(m)^2, 4*ts_delta(m)^3, 5*ts_delta(m)^4, ...
                      6*ts_delta(m)^5, 7*ts_delta(m)^6; ...
                      0, 0, 2, 6*ts_delta(m), 12*ts_delta(m)^2, 20*ts_delta(m)^3, ...
                      30*ts_delta(m)^4, 42*ts_delta(m)^5; ...
                      0, 0, 0, 6, 24*ts_delta(m), 60*ts_delta(m)^2, ...
                      120*ts_delta(m)^3, 210*ts_delta(m)^4];

% quadprog Quadratic programming. 
%             min 0.5*x'*H*x + f'*x   subject to:  A*x <= b 
%              x    
%    X = quadprog(H,f,A,b,Aeq,beq) solves the problem above while
%    additionally satisfying the equality constraints Aeq*x = beq. (Set A=[]
%    and B=[] if no inequalities exist.)
%% QP
f=zeros(8*m,1);
P(:,1) = quadprog(Q_dia,f,[],[],A,d(:,1));
P(:,2) = quadprog(Q_dia,f,[],[],A,d(:,2));
P(:,3) = quadprog(Q_dia,f,[],[],A,d(:,3));
%% visualize the 2D grid map
subplot(h);
% start point
plot3(map(1, 1)-0.5, map(1, 2)-0.5, map(1, 3)-0.5, 'k.');
hold on;
plot3(path(:,1),path(:,2),path(:,3),'ko','markerfacecolor','k');
hold on;
% obstacles
for obs_cnt = 2: size(map, 1) - 1
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.2 map(obs_cnt, 2)-0.8], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.8 map(obs_cnt, 2)-0.2], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    [X,Y,Z] = cylinder(0.4);
    %Z(2, :) = 5;
    X(:, :) = X(:, :) + map(obs_cnt, 1) - 0.5;
    Y(:, :) = Y(:, :) + map(obs_cnt, 2) - 0.5;
    %plz be tiny
    Z(:, :) = Z(:, :) + map(obs_cnt, 3) -0.7;

    mesh(X,Y,Z,'edgecolor', [0.7, 0.7, 0.7], 'facecolor', [0.7,0.7,0.7]); 
    grid minor
    set(gca,'xtick',[-100:1:100])
    set(gca,'ytick',[-100:1:100])
    grid off;
    grid on;       
    axis equal;        
    axis ([-1 6 -1 10 0 4]);
    hold on;
end
%% target point
plot3(map(obs_cnt+1, 1)-0.5, map(obs_cnt+1, 2)-0.5, map(obs_cnt+1, 3)-0.5, 'r*');
hold on;

else % output desired trajectory here (given time)
% s_des = zeros(13,1);
% [1:6]: x, y, z, xdot, ydot, zdot
% [7:10]: quaternion, qw,qx,qy,qz
% [11:13]: oemga: wx, wy, wz   
%% find t 
s_des = zeros(13,1);
if t >ts(end) %hover at the end position
    s_des(1:3)=xyz;
    s_des(7:10) =[1;0;0;0];
    return;
end
ind = 1;  
time = 0;
for i = 1:length(ts_delta)
   if t>=ts(i) && t<=ts(i+1)
       ind = i;
       time = ts(i);
       break
   end
end
T = t - time;

%T = t-ts(ind);
p = P(8*ind-7:8*ind,:);
%% poly

s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
xyz = s_des(1:3);
s_des(4:6) = [0,1,2*T,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*p;
s_des(7:10) =[1;0;0;0];
end

end



