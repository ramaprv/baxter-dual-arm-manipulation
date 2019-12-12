%Forward velocity kinematics (FVK) for the Baxter robot.
%Note: FVK gives us end effector velocity
clc; clear all; close all;

syms t1 t2 t3 t4 t5 t6 t7;
syms d1 a1 d3 a3 d5 a5 d7;

% d1 = 270.35; a1 = 69;
% d3 = 364.35; a3 = -69;
% d5 = 374.29; a5 = -10;
% d7 = 229.525;

A1 = calcdh_deg(t1*180/pi, d1, a1, -90);
A2 = calcdh_deg((t2-90)*180/pi, 0, 0, -90);
A3 = calcdh_deg(t3*180/pi, d3, -a3, 90);
A4 = calcdh_deg(t4*180/pi, 0, 0, -90);
A5 = calcdh_deg(t5*180/pi, d5, -a5, 90);
A6 = calcdh_deg(t6*180/pi, 0, 0, -90);
A7 = calcdh_deg(t7*180/pi, d7, 0, 0);

%Transformation matrix 
T = A1 * A2 * A3 * A4 * A5 * A6 * A7;
T = simplify(T)

xt = T(1:3, 4);

%Velocity Jacobian elements
%Joint 1
v1 = diff(xt, t1);
v1 = simplify(v1);
%Joint 2
v2 = diff(xt, t2);
v2 = simplify(v2);
%Joint 3
v3 = diff(xt, t3);
v3 = simplify(v3);
%Joint 4
v4 = diff(xt,t4);
v4 = simplify(v4);
%Joint 5
v5 = diff(xt, t5);
v5 = simplify(v5);
%Joint 6
v6 = diff(xt, t6);
v6 = simplify(v6);
%Joint 7
v7 = diff(xt, t7);
v7 = simplify(v7);
%Velocity Jacobian
Jv = [v1 v2 v3 v4 v5 v6 v7];

%Angular velocity Jacobian elements
k = [0; 0; 1];
%Joint 1
w1 = k
%Joint 2
R = A1(1:3, 1:3)
w2 = R * k
%Joint 3
A = A1 * A2;
R = A(1:3, 1:3);
w3 = R * k;
%Joint 4
A = A1 * A2 * A3;
R = A(1:3, 1:3);
w4 = R * k;
%Joint 5
A = A1 * A2 * A3 * A4;
R = A(1:3, 1:3);
w5 = R * k;
%Joint 6
A = A1 * A2 * A3 * A4 * A5;
R = A(1:3, 1:3);
w6 = R * k;
%Joint 7
A = A1 * A2 * A3 * A4 * A5 * A6;
R = A(1:3, 1:3);
w7 = R * k;
%Angular velocity Jacobian
Jw = [w1 w2 w3 w4 w5 w6 w7];

%Final Jacobian
J = [Jv; Jw]

%For Inverse velocity kinematics(IVK) of the Baxter robot.
%Note: IVK gives us joint velocities
%J_inv = pinv(J) , Moore - penrose pseudo inverse for a non - square matrix
