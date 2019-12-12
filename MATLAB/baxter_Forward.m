%Forward kinematics for the baxter robot, robot dynamics project work

clc; clear all; close all;

syms t1 t2 t3 t4 t5 t6 t7;
syms d1 a1 d3 a3 d5 a5 d7;

% d1 = 270.35; a1 = 69;
% d3 = 364.35; a3 = -69;
% d5 = 374.29; a5 = -10;
% d7 = 229.525;

A1 = calcdh_deg(t1*180/pi, d1, a1, -90)
A2 = calcdh_deg((t2-90)*180/pi, 0, 0, -90)
A3 = calcdh_deg(t3*180/pi, d3, -a3, 90)
A4 = calcdh_deg(t4*180/pi, 0, 0, -90)
A5 = calcdh_deg(t5*180/pi, d5, -a5, 90)
A6 = calcdh_deg(t6*180/pi, 0, 0, -90)
A7 = calcdh_deg(t7*180/pi, d7, 0, 0)

%Transformation matrix 
T = A1 * A2 * A3 * A4 * A5 * A6 * A7;
T = simplify(T)
x = T(1,4)
y = T(2,4)
z = T(3,4)
