clc;clear all;close all;
%%
%Performing forward kinematics for the robot
syms t1(t) t2(t) t3(t) t4(t) t5(t) t6(t) t7(t);
syms m1 m2 m3 m4 m5 m6 m7;
syms d1 a1 d3 a3 d5 a5 d7;
syms g

d1 = 270.35; a1 = 69; d3 = 364.35; a3 = 69; d5 = 374.29; a5 = 10; 
d7 = 229.525;  

A1 = calcdh_deg(t1*180/pi, d1, a1, -90);
A2 = calcdh_deg((t2-90)*180/pi, 0, 0, -90);
A3 = calcdh_deg(t3*180/pi, d3, -a3, 90);
A4 = calcdh_deg(t4*180/pi, 0, 0, -90);
A5 = calcdh_deg(t5*180/pi, d5, -a5, 90);
A6 = calcdh_deg(t6*180/pi, 0, 0, -90);
A7 = calcdh_deg(t7*180/pi, d7, 0, 0);

T = A1 * A2 * A3 * A4 * A5 * A6 * A7;
T = simplify(T)

%%
%Obtaining positions of every mass

%Position of mass 1 
A = A1;
pos1 = A1 *  [0; 0; 0; 1]
%Position of mass 2
A = A * A2;
pos2 = A * [0; 0; 0; 1];
%Position of mass 3
A = A * A3;
pos3 = A * [0; 0; 0; 1];
%Position of mass 4
A = A * A4;
pos4 = A * [0; 0; 0; 1];
%Position of mass 5
A = A * A5;
pos5 = A * [0; 0; 0; 1];
%Position of mass 6
A = A * A6;
pos6 = A * [0; 0; 0; 1];
%Position of mass 7
A = A * A7;
pos7 = A * [0; 0; 0; 1]; 

%%
%Finding individual velocities

v1 = diff(pos1, t);
v2 = diff(pos2, t);
v3 = diff(pos3, t);
v4 = diff(pos4, t);
v5 = diff(pos5, t);
v6 = diff(pos6, t);
v7 = diff(pos7,t);

%
%Finding the Kinematic energies

k1 = (1/2) * m1 * (transpose(v1) * v1);
k2 = (1/2) * m2 * (transpose(v2) * v2);
k3 = (1/2) * m3 * (transpose(v3) * v3);
k4 = (1/2) * m4 * (transpose(v4) * v4);
k5 = (1/2) * m5 * (transpose(v5) * v5);
k6 = (1/2) * m6 * (transpose(v6) * v6);
k7 = (1/2) * m7 * (transpose(v7) * v7);

%Total kinetic energy 
k = k1 + k2 + k3 + k4 + k5 + k6 + k7

%%
%Finding the potential energies

p1 = m1 * g * (transpose(pos1) * [0; 0; 1; 0]);
p2 = m2 * g * (transpose(pos2) * [0; 0; 1; 0]);
p3 = m3 * g * (transpose(pos3) * [0; 0; 1; 0]);
p4 = m4 * g * (transpose(pos4) * [0; 0; 1; 0]);
p5 = m5 * g * (transpose(pos5) * [0; 0; 1; 0]);
p6 = m6 * g * (transpose(pos6) * [0; 0; 1; 0]);
p7 = m7 * g * (transpose(pos7) * [0; 0; 1; 0]);

%Total potential energy 
p = p1 + p2 + p3 + p4 + p5 + p6 + p7
 %p = simplify(p)

%%
%Finding lagrangian
L = k - p

% 
% %%
% %Finding partial derivatives
% 
% %First substituting so that diff(t),t is differentiable
syms th1_d th2_d th3_d th4_d th5_d th6_d th7_d;
syms th1 th2 th3 th4 th5 th6 th7

L = subs(L,[diff(t1(t),t), diff(t2(t),t),diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t) t7(t)],[th1_d,...
    th2_d, th3_d, th4_d, th5_d, th6_d, th7_d,th1, th2,...
    th3, th4, th5, th6, th7]);
% L = subs(L,[t1(t), t2(t), t3(t), t4(t), t5(t), t6(t) t7(t)],[th1, th2,...
%     th3, th4, th5, th6, th7]);
% 
 partial_wrt_th1 = diff(L, th1)
partial_wrt_th2 = diff(L, th2)
partial_wrt_th3 = diff(L, th3)
partial_wrt_th4 = diff(L, th4)
partial_wrt_th5 = diff(L, th5)
partial_wrt_th6 = diff(L, th6)
partial_wrt_th7 = diff(L, th7)

partial_wrt_th1_d = diff(L, th1_d)
partial_wrt_th2_d = diff(L, th2_d)
partial_wrt_th3_d = diff(L, th3_d)
partial_wrt_th4_d = diff(L, th4_d)
partial_wrt_th5_d = diff(L, th5_d)
partial_wrt_th6_d = diff(L, th6_d)
partial_wrt_th7_d = diff(L, th7_d)
% 
%Reversing the substitutions to get time derivatives

partial_wrt_th1_d = subs(partial_wrt_th1_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);

partial_wrt_th2_d = subs(partial_wrt_th2_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);


partial_wrt_th3_d = subs(partial_wrt_th3_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);

partial_wrt_th4_d = subs(partial_wrt_th4_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);


partial_wrt_th5_d = subs(partial_wrt_th5_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);


partial_wrt_th6_d = subs(partial_wrt_th6_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);

partial_wrt_th7_d = subs(partial_wrt_th7_d,[th1_d, th2_d, th3_d, th4_d,...
    th5_d, th6_d, th7_d,th1, th2, th3, th4, th5, th6,...
    th7],[diff(t1(t),t), diff(t2(t),t) diff(t3(t),t),...
    diff(t4(t),t),diff(t5(t),t),diff(t6(t),t),diff(t7(t),t),t1(t), t2(t), t3(t), t4(t), t5(t), t6(t), t7(t)]);

%Finding time derivatives
time_derivative_tau1 = diff(partial_wrt_th1_d, t)
time_derivative_tau2 = diff(partial_wrt_th2_d, t)
time_derivative_tau3 = diff(partial_wrt_th3_d, t)
time_derivative_tau4 = diff(partial_wrt_th4_d, t)
time_derivative_tau5 = diff(partial_wrt_th5_d, t)
time_derivative_tau6 = diff(partial_wrt_th6_d, t)
time_derivative_tau7 = diff(partial_wrt_th7_d, t)

%%
%Finding individual torques
tau1 = time_derivative_tau1 - partial_wrt_th1;
tau2 = time_derivative_tau2 - partial_wrt_th2;
tau3 = time_derivative_tau3 - partial_wrt_th3;
tau4 = time_derivative_tau4 - partial_wrt_th4;
tau5 = time_derivative_tau5 - partial_wrt_th5;
tau6 = time_derivative_tau6 - partial_wrt_th6;
tau7 = time_derivative_tau7 - partial_wrt_th7;

%The Torque vector
Tau =  [tau1; tau2; tau3; tau4; tau5; tau6; tau7]
%Tau_simp = simplify (Tau)

%%
%Performing substitutions to obtain M, C, G matrices

syms th1_dd th2_dd th3_dd th4_dd th5_dd th6_dd th7_dd
Tau = subs(Tau,[diff(t1(t),t,t),diff(t2(t),t,t),diff(t3(t),t,t),...
    diff(t4(t),t,t),diff(t5(t),t,t),diff(t6(t),t,t),diff(t7(t),t,t),diff(t1(t),t),diff(t2(t),t),diff(t3(t),t),diff(t4(t),t),...
    diff(t5(t),t),diff(t6(t),t),diff(t7(t),t)],[...
    th1_dd, th2_dd, th3_dd, th4_dd, th5_dd, th6_dd, th7_dd,th1_d, th2_d, th3_d,...
    th4_d, th5_d, th6_d, th7_d]);

disp('The updated Tau')
disp(Tau)

%%
%The Inertia matrix(M)
IM = equationsToMatrix(Tau,[th1_dd th2_dd th3_dd th4_dd th5_dd th6_dd...
    th7_dd])

%%
%The Gravity matrix(G)
GM = equationsToMatrix(Tau, g)

%%
%The Coriolis matrix(C)

CM = (Tau - IM*[th1_dd;th2_dd;th3_dd;th4_dd;th5_dd;th6_dd;th7_dd] - GM*g)









