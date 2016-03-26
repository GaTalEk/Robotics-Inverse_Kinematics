function [ W ] = solve_wrist_inverse( coord3, od, kd, jd )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
C3 = coord3(1:3,1:3);

% Solve inverse wrist cinematics
% see Slide 5, Inverse Kinematics
i3 = C3(:,1);
j3 = C3(:,2);
k3 = C3(:,3);

k6 = kd;

k4_1 = cross(k3,k6)/norm(cross(k3,k6));
k4_2 = -k4_1;

% Find theta4 (Kahan P1)
theta4_1 = 2*atan(norm(k4_1-i3)/norm(k4_1 + i3)) * sign(k4_1'*i3);
theta4_2 = 2*atan(norm(k4_2-i3)/norm(k4_2 + i3)) * sign(k4_2'*i3);

[theta5_1, theta6_1] = solve_wrist_second_half(coord3, theta4_1, kd, jd);
[theta5_2, theta6_2] = solve_wrist_second_half(coord3, theta4_2, kd, jd);
    
W = [theta4_1, theta5_1, theta6_1; theta4_2, theta5_2, theta6_2];
end

function [theta5, theta6] = solve_wrist_second_half(coord3, theta4, kd, jd)

coord4 = coord3 * dh_transform(pi/2 + theta4, 0, 0, -pi/2);

C4 = coord4(1:3,1:3);

k5 = kd;
i4 = C4(:,1);
k4 = C4(:,3);
k3 = coord3(1:3,3);

theta5 = 2*atan(norm(k5 - (-i4))/norm(k5 + (-i4))) * sign(k5'*cross(k4,k3));

coord5 = coord4 * dh_transform(-pi/2 + theta5, 0, 0, pi/2);
C5 = coord5(1:3,1:3);

i5 = C5(:,1);
j5 = C5(:,2);

theta6 = 2*atan(norm(jd - j5)/norm(jd + j5)) * sign(jd'*i5);

end

