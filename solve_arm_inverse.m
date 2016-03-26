function [ A ] = solve_arm_inverse( coord0, o3 )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

l1 = 1;
l2 = 1;
l3 = 0.5;
l6 = 0.2;

o2 = o3;

o2(3) = l3+l6;

d3 = o2(3) - o3(3) - l3;

% Solve inverse arm kinematics
% see Slide 1 of Inverse Kinematics slides

o0 = coord0(1:3,4);

u = o2 - o0;
l = sqrt(u(1)*u(1) + u(2)*u(2));

theta2_1 = pi - acos((-l^2 + l1^2 + l2^2)/(2*l1*l2));

alpha = atan(-u(1)/u(2));

% the atan always returns a value between -pi/2 and pi/2,
% so you have to check if the point you want to reach is in the left half
if u(2) < 0
    alpha = alpha + pi;
end

beta = acos((-l2^2 + l1^2 + l^2)/(2*l1*l));

theta1_1 = alpha - beta;

% other solution

theta1_2 = alpha + beta;
theta2_2 = -theta2_1;

% there are infinte solutions for theta1 if the gripper is supposed
% to be at the origin ([0; 0; 0])
if isnan(u(1)/u(2))
    theta1_1 = 0;
    theta2_1 = pi;
end

a1 = [ theta1_1, theta2_1, d3];
a2 = [ theta1_2, theta2_2, d3];

A = [a1;a2];

end

