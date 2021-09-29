clc
close all

addpath('./../matlab_utils')

syms th1 th2 th3 l1 l2 l3 phi

% End Effector Coordinate Functions
x_ee = l1*cos(th1) + l2*cos(th1+th2) - l3*cos(phi-th1);
y_ee = l1*sin(th1) + l2*sin(th1+th2) + l3*sin(phi-th1);

% Taking the Jacobian
J = jacobian([x_ee; y_ee], [th1; th2]);
disp(J);

% Taking the transpose of the Jacobian
J_T = J.';
disp(J_T);