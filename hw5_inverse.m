l1 = 1;
l2 = 1;
l3 = 0.5;
l6 = 0.2;

i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

kx = [0 0 0;
      0 0 -1;
      0 1 0];

o0 = [0; 0; 0];
C0 = eye(3);

coord0 = [C0 o0;
          zeros(1,3) 1];

% Ask for o6, approach vector and sliding vector
od = input('Enter od ([od_i; od_j; od_k]): ');
kd = input('Enter approach vector (kd) ([kd_i; kd_j; kd_k]): ');
jd = input('Enter sliding vector (jd) ([jd_i; jd_j; jd_k]): ');
id = -cross(kd, jd);

Cd = [id jd kd];

% Calculate wrist center

o3 = od - Cd*l6*k;

o2 = o3;

o2(3) = l3+l6;

d3 = o2(3) - o3(3) - l3;

% Solve inverse arm kinematics
% see Slide 1 of Inverse Kinematics slides
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
    disp('Infinite solutions for theta1, just displaying one with theta1 = 0');
    theta1_1 = 0;
    theta2_1 = pi;
    
    theta1_2 = 0;
    theta2_2 = pi;
end

a1 = [ theta1_1, theta2_1, d3];
a1 = [a1;a1];
a2 = [ theta1_2, theta2_2, d3];
a2 = [a2;a2];

% Get wrist base frame by direct kinematics
coord3_1 = solve_arm_forward(coord0, theta1_1, theta2_1, d3);
coord3_2 = solve_arm_forward(coord0, theta1_2, theta2_2, d3);

w1 = solve_wrist_inverse(coord3_1,  od, kd, jd);
w2 = solve_wrist_inverse(coord3_2,  od, kd, jd);

result = [ a1 w1; a2 w2]

