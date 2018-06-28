%% Robotics Project

%% Question 0 - Clear Workspace
clear
close all
clc

%% Question 1 - Forward Kinematics

syms theta_9 theta_10 theta_11 theta_12 theta_13
syms d_9 d_10 d_11 d_12 d_13
syms a_9 a_10 a_11 a_12 a_13
syms alpha_9 alpha_10 alpha_11 alpha_12 alpha_13

%Declare the D-H numeric values
d_9 = 0;
a_9 = 0;
alpha_9 = deg2rad(90);

theta_10 = deg2rad(90);
a_10 = 0;
alpha_10 = deg2rad(0);

d_11 = 0;
a_11 = 0;
alpha_11 = deg2rad(-90);

d_12 = 0;
a_12 = 10;
alpha_12 = deg2rad(-90);

d_13 = 0;
a_13 = 8;
alpha_13 = deg2rad(-90);

theta = {theta_9, theta_10, theta_11, theta_12, theta_13};
d = {d_9, d_10, d_11, d_12, d_13};
a = {a_9, a_10, a_11, a_12, a_13};
alpha = {alpha_9, alpha_10, alpha_11, alpha_12, alpha_13};


%Calculate A between subsequent frames
for i = 1:5
    A{i} = [cos(theta{i}), -sin(theta{i})*cos(alpha{i}),  sin(theta{i})*sin(alpha{i}), a{i}*cos(theta{i});
            sin(theta{i}),  cos(theta{i})*cos(alpha{i}), -cos(theta{i})*sin(alpha{i}), a{i}*sin(theta{i});
            0            ,  sin(alpha{i})              ,  cos(alpha{i})              , d{i}              ;
            0            ,  0                          ,  0                          , 1                ];
end

T{1} = A{1};
T{2} = T{1}*A{2};
T{3} = T{2}*A{3};
T{4} = T{3}*A{4};
T{5} = T{4}*A{5}; %This is the complete transform from 8 to 13

%Export the transform to a MATLAB function to be called later
%This is the FORWARD KINEMATICS (fk) function
matlabFunction(T{5},'File','fk','Vars',[theta_9, d_10, theta_11, theta_12, theta_13]);

%These are the forward kinematics functions of the in-between joints
matlabFunction(T{1},'File','T1','Vars',[theta_9, d_10, theta_11, theta_12, theta_13]);
matlabFunction(T{2},'File','T2','Vars',[theta_9, d_10, theta_11, theta_12, theta_13]);
matlabFunction(T{3},'File','T3','Vars',[theta_9, d_10, theta_11, theta_12, theta_13]);
matlabFunction(T{4},'File','T4','Vars',[theta_9, d_10, theta_11, theta_12, theta_13]);

%% Question 2 - Robot Workspace 

%Define the joint limits
theta_9_max  = deg2rad(60);
d_10_max     = 200;
theta_11_max = deg2rad(60);
theta_12_max = deg2rad(45);
theta_13_max = deg2rad(30);

%Define iteration step
angle_step = deg2rad(10);
dist_step  = 10;

count = 0;

tic
%Calculate the workspace
for theta_9 = 0 : angle_step : theta_9_max
    for d_10 = 0 : dist_step : d_10_max
        for theta_11 = 0 : angle_step : theta_11_max
            for theta_12 = 0 : angle_step : theta_12_max
                for theta_13 = 0 : angle_step : theta_13_max
                %For Gripper
                count = count + 1;
                T_robot = fk(theta_9,d_10,theta_11,theta_12,theta_13);
                x(count) = (T_robot(1,4));
                y(count) = (T_robot(2,4));
                z(count) = (T_robot(3,4));
                %For Joint 9
                count = count + 1;
                T_robot = T1(theta_9,d_10,theta_11,theta_12,theta_13);
                x(count) = (T_robot(1,4));
                y(count) = (T_robot(2,4));
                z(count) = (T_robot(3,4));
                %For Joint 10
                count = count + 1;
                T_robot = T2(theta_9,d_10,theta_11,theta_12,theta_13);
                x(count) = (T_robot(1,4));
                y(count) = (T_robot(2,4));
                z(count) = (T_robot(3,4));
                %For Joint 11
                count = count + 1;
                T_robot = T3(theta_9,d_10,theta_11,theta_12,theta_13);
                x(count) = (T_robot(1,4));
                y(count) = (T_robot(2,4));
                z(count) = (T_robot(3,4));
                %For Joint 12
                count = count + 1;
                T_robot = T4(theta_9,d_10,theta_11,theta_12,theta_13);
                x(count) = (T_robot(1,4));
                y(count) = (T_robot(2,4));
                z(count) = (T_robot(3,4));
                end
            end
        end
    end
end
str = ['Time needed to calculate the workspace: ', num2str(toc), ' s.'];
disp(str)
% workspace_calculation_time = toc

%Find minima/maxima per axis
xmin = min(x);
xmax = max(x);
ymin = min(y);
ymax = max(y);
zmin = min(z);
zmax = max(z);

% Plot workspace as boundary
disp('Plotting workspace, please wait...')
k = boundary(x',y',z');
figure(1)
trisurf(k,x',y',z','Facecolor','red','FaceAlpha',0.1)
str = ['Robot Workspace (Xmin=',num2str(xmin),', Xmax=',num2str(xmax),', Ymin=',num2str(ymin),', Ymax=',num2str(ymax),', Zmin=',num2str(zmin),', Zmax=',num2str(zmax)];
title(str)
% title('Robot Workspace')
xlabel('Global X-axis [mm]')
ylabel('Global Y-axis [mm]')
zlabel('Global Z-axis [mm]')
ylim([-220 30])
legend('Robot Workspace')
disp('Workspace plotted.')


%% Questions 3/4 - Inverse Kinematics and Spiral Trajectory

%Generate the spiral
xc=75;
yc=-100;
zc=13;
r=50;

%Define slope of spiral
d=0.1;
%Define timestep
dt=1/(2*36);
%Define period of spiral
Ts=1;

%Generate (2/dt)+1 points in time
t=0:dt:(2*Ts);

xs = xc + r*cos(2*pi*t/Ts);
ys = yc + r*sin(2*pi*t/Ts);
zs = zc + d*2*pi*t/Ts;

%Plot the spiral
figure(2)
subplot(1,2,1);
%Plot the spiral inside the workspace
plot3(xs, ys, zs);
legend('Desired Trajectory')
title('Desired Trajectory')
xlabel('Global X-axis [mm]')
ylabel('Global Y-axis [mm]')
zlabel('Global Z-axis [mm]')


%Plot the workspace again to include the spiral
% figure(3)
subplot(1,2,2);
hold on
trisurf(k,x',y',z','Facecolor','red','FaceAlpha',0.1,'LineStyle','none')
str = ['Robot Workspace (Xmin=',num2str(xmin),', Xmax=',num2str(xmax),', Ymin=',num2str(ymin),', Ymax=',num2str(ymax),', Zmin=',num2str(zmin),', Zmax=',num2str(zmax)];
title(str)
% title('Robot Workspace')
xlabel('Global X-axis [mm]')
ylabel('Global Y-axis [mm]')
zlabel('Global Z-axis [mm]')

%Plot the spiral inside the workspace
plot3(xs, ys, zs, 'LineWidth', 2);
legend('Robot Workspace','Desired Trajectory')
title('Desired Trajectory inside the Workspace')
hold off

% Solve the Inverse Kinematics for all the spiral points

%Define desired gripper XYZ setpoint as global variable
global p_des

%Define the joint limits
theta_9_max  = deg2rad(60);
d_10_max     = 200;
theta_11_max = deg2rad(60);
theta_12_max = deg2rad(45);
theta_13_max = deg2rad(30);

%Create the lower/upper-bound constraint vectors
lb = [0,0,0,0,0];
ub = [theta_9_max, d_10_max, theta_11_max, theta_12_max, theta_13_max];

%Unused parameters defined as empty
A = [];
b = [];
Aeq = [];
beq = [];

%Conservative initial guess as "center" of joint limits
q0 = ub/2;

%Suppress most output from the solver
options = optimoptions('fmincon','Display','notify');

%Initialize matrix to store the results
q = zeros(length(t),5);

tic
for i = 1:length(t)
    %Desired gripper XYZ setpoint in global coordinates
    p_des = [xs(i), ys(i), zs(i)];

    %Solve the IK as a nonlinear optimization problem
    [q_out,costval,exitflag] = fmincon(@costfun,q0,A,b,Aeq,beq,lb,ub,[],options);
    
    %Save the configuration
    q(i,:) = q_out;
    
    %Update the initial guess for next step
    q0 = q_out;
    
    txt = ['Solving for point ',num2str(i),'.'];
    disp(txt)
end
toc

figure(3)
%Joint 1
subplot(5,1,1);
plot(rad2deg(q(:,1)),'.','markers',12)
title('Joint 1 (Elbow Rotation)')
ylabel('[deg]')
xlim([0 length(t)])
y = 60;
line([0,length(t)],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 2
subplot(5,1,2);
plot(q(:,2),'.','markers',12);
title('Joint 2 (Elbow Extension/Retraction)')
ylabel('[mm]')
xlim([0 length(t)])
ylim([0 220])
y = 200;
line([0,length(t)],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 3
subplot(5,1,3);
plot(rad2deg(q(:,3)),'.','markers',12);
title('Joint 3 (Wrist Roll)')
ylabel('[deg]')
xlim([0 length(t)])
y = 60;
line([0,length(t)],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 4
subplot(5,1,4);
plot(rad2deg(q(:,4)),'.','markers',12);
title('Joint 4 (Wrist Pitch)')
ylabel('[deg]')
xlim([0 length(t)])
y = 45;
line([0,length(t)],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 5
subplot(5,1,5);
plot(rad2deg(q(:,5)),'.','markers',12);
title('Joint 5 (Wrist Yaw)')
xlabel('Spiral Points')
ylabel('[deg]')
xlim([0 length(t)])
y = 30;
line([0,length(t)],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

% Plot the actual vs desired gripper motion
for i = 1:length(q)
    point = fk(q(i,1),q(i,2),q(i,3),q(i,4),q(i,5));
    xs_gripper(i) = point(1,4);
    ys_gripper(i) = point(2,4);
    zs_gripper(i) = point(3,4);
end

%Plot the spiral
figure(4)
hold on
plot3(xs_gripper, ys_gripper, zs_gripper, '.','markers',12);
plot3(xs, ys, zs);
xlabel('x'); 
ylabel('y'); 
zlabel('z'); 
title('Inverse Kinematics Verification');
legend('Inverse-Kinematics-Based Motion','Desired Motion')
grid;
hold off

%% Question 5 - Cubic Splines Interpolation
%Create a cubic spline for every two subsequent points 

%Number of calculated configurations
n = length(q);

%Number of total points per cubic spline (for more accurate visualization)
m = 5;

%Row counter
row = 1;

for i = 1:(n-1)
    %Define the time vector for the current spline
    q_start = q(i,:);
    q_end   = q(i+1,:);
    t_spline = linspace(t(i),t(i+1),m);

    M = [1,   t(i),   t(i)^2,     t(i)^3;
         0,      1,   2*t(i),   3*t(i)^2;
         1, t(i+1), t(i+1)^2,   t(i+1)^3;
         0,      1, 2*t(i+1), 3*t(i+1)^2];
     
    %Calculate the coefficients of the splines of EACH JOINT 
    for j = 1:5 %5 joints
        a{j} = M \ [q_start(j); 0; q_end(j); 0];
    end

    %Form the spline
    for k = 1:m-1
        for j = 1:5
          q_cubic_spline(row,j) = a{j}(1) + a{j}(2)*t_spline(k) + a{j}(3)*(t_spline(k)^2) + a{j}(4)*(t_spline(k)^3);
          t_cubic_spline(row) = t_spline(k);
        end
        row = row + 1;
    end
end

%Add the last point of the last curve that was intentionally omitted
q_cubic_spline(row,:) = q(end,:);
t_cubic_spline(row) = t_spline(k);

% Plots

figure(5)
%Joint 1
subplot(5,1,1);
plot(t_cubic_spline,rad2deg(q_cubic_spline(:,1)),'.','markers',12)
title('Joint 1 (Elbow Rotation)')
ylabel('[deg]')
xlim([0 t_cubic_spline(end)+0.01])
y = 60;
line([0,t_cubic_spline(end)+0.01],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 2
subplot(5,1,2);
plot(t_cubic_spline,q_cubic_spline(:,2),'.','markers',12);
title('Joint 2 (Elbow Extension/Retraction)')
ylabel('[mm]')
xlim([0 t_cubic_spline(end)+0.01])
ylim([0 220])
y = 200;
line([0,t_cubic_spline(end)+0.01],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 3
subplot(5,1,3);
plot(t_cubic_spline,rad2deg(q_cubic_spline(:,3)),'.','markers',12);
title('Joint 3 (Wrist Roll)')
ylabel('[deg]')
xlim([0 t_cubic_spline(end)+0.01])
y = 60;
line([0,t_cubic_spline(end)+0.01],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 4
subplot(5,1,4);
plot(t_cubic_spline,rad2deg(q_cubic_spline(:,4)),'.','markers',12);
title('Joint 4 (Wrist Pitch)')
ylabel('[deg]')
xlim([0 t_cubic_spline(end)+0.01])
y = 45;
line([0,t_cubic_spline(end)+0.01],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

%Joint 5
subplot(5,1,5);
plot(t_cubic_spline,rad2deg(q_cubic_spline(:,5)),'.','markers',12);
title('Joint 5 (Wrist Yaw)')
xlabel('Time [s]')
ylabel('[deg]')
xlim([0 t_cubic_spline(end)+0.01])
y = 30;
line([0,t_cubic_spline(end)+0.01],[y,y],'Color','red','LineStyle','--','LineWidth',1.5)
legend('Joint Value','Joint Limit')
grid on

% Plot Gripper Trajectory

for i = 1:length(q_cubic_spline)
    point = fk(q_cubic_spline(i,1),q_cubic_spline(i,2),q_cubic_spline(i,3),q_cubic_spline(i,4),q_cubic_spline(i,5));
    xs_gripper_new(i) = point(1,4);
    ys_gripper_new(i) = point(2,4);
    zs_gripper_new(i) = point(3,4);
end

figure(6)
%Gripper x-motion (global frame)
subplot(3,1,1);
plot(t_cubic_spline,xs_gripper_new,'.-','markers',12);
title('Tool X-motion (Global Frame)')
ylabel('[mm]')
xlim([0 t_cubic_spline(end)+0.01])
y = 60;
grid on

%Gripper y-motion (global frame)
subplot(3,1,2);
plot(t_cubic_spline,ys_gripper_new,'.','markers',12);
title('Tool Y-motion (Global Frame)')
ylabel('[mm]')
xlim([0 t_cubic_spline(end)+0.01])
ylim([-170 -20])
y = 45;
grid on

%Gripper z-motion (global frame)
subplot(3,1,3);
plot(t_cubic_spline,zs_gripper_new,'.','markers',12);
title('Tool Z-motion (Global Frame)')
xlabel('Time [s]')
ylabel('[mm]')
xlim([0 t_cubic_spline(end)+0.01])
ylim([12 15])
y = 30;
grid on
