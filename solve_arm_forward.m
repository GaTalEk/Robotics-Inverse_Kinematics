function [ coord_wrist ] = solve_arm_forward( coord0, theta1, theta2, d3 )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

l1 = 1;
l2 = 1;
l3 = 0.5;
l6 = 0.2;

coord_wrist = coord0 * dh_transform(pi/2 + theta1, l3 + l6, l1, 0) ...
                     * dh_transform(theta2, 0, l2, pi) ...
                     * dh_transform(-pi/2, l3 + d3, 0, -pi/2);

end

