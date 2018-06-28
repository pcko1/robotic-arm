function [costval] = costfun(q)
%This function calculates the cost function to be used for the optimization
%problem of inverse kinematics for the 5-DOF robot.

theta_9  = q(1);
d_10     = q(2);
theta_11 = q(3);
theta_12 = q(4);
theta_13 = q(5);

%Read global setpoint from workspace
global p_des

%Calculate the current configuration using forward kinematics 
Tcurr = fk(theta_9,d_10,theta_11,theta_12,theta_13);

%Isolate the XYZ coordinates of the current configuration
p = [Tcurr(1,4), Tcurr(2,4), Tcurr(3,4)]; 

%Find the error as Euclidean distance
% costval = sqrt( (p(1)-p_des(1))^2 + (p(2)-p_des(2))^2 + (p(3)-p_des(3))^2);
costval = sqrt(sum((p-p_des).^2));

end
