%Forward kinematics for the updated baxter robot, robot dynamics project work

clc; clear all; close all;

syms t1 t2 t3 t4 t5 t6;
syms d1 l3 d5 d7;

% d1 = 270.35;
% l3 = 438.80;
% d5 = 374.29;
% d7 = 229.525;

A1 = calcdh_deg(t1*180/pi, d1, 0, -90);
A2 = calcdh_deg(t2*180/pi, 0, l3, 0);
A3 = calcdh_deg((t3-90)*180/pi, 0, 0, -90);
A4 = calcdh_deg(t4*180/pi, d5, 0, 90);
A5 = calcdh_deg(t5*180/pi, 0, 0, -90);
A6 = calcdh_deg(t6*180/pi, d7, 0, 0);

%Transformation matrix 
T = A1 * A2 * A3 * A4 * A5 * A6;
T = simplify(T)

%To perform inverse orientation kinematics
R_0wrt6 = T(1:3,1:3);
A_0wrt3 = A1 * A2 * A3;
R_0wrt3 = A_0wrt3(1:3,1:3);

%Euler angle parametrization
R_z_by_t4 = [cos(t4) -sin(t4) 0;...
            sin(t4) cos(t4) 0;...
            0 0 1];
R_y_by_t5 = [cos(t5) 0 sin(t5);...
            0 1 0;...
            -sin(t5) 0 cos(t5)];
R_z_by_t6 = [cos(t6) -sin(t6) 0;...
            sin(t6) cos(t6) 0;...
            0 0 1];
%LHS
R_3wrt6 = R_z_by_t4 * R_y_by_t5 * R_z_by_t6

%RHS 
R = transpose(R_0wrt3) * R_0wrt6;
R = simplify(R)

%This forms a condition of case 1 where r13 and r23 are != 0.
cos(t5) = R(3,3)
sin(t5) = sqrt(1 - (cos(t5))^2)

%Considering plus solution for sin(t5)
t5 = atan2(cos(t5), sin(t5))
t4 = atan2(R(1,3), R(2,3))
t6 = atan2(-R(3,1), R(3,2))
