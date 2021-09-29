clc
close all

addpath('./../matlab_utils')

%%
l1 = 0.1; l2 = 0.1; l3 = 0.15; rho = 0.8;
q = [0.6; 0.3];

x_des = [0.06; 0.14];

fx_1 = @(q)[l1*cos(q(1)); l1*sin(q(2))];
fx_2 = @(q)[l1*cos(q(1)) + l2*cos(q(1)+q(2));l1*sin(q(1)) + l2*sin(q(1)+q(2))];
fx_ee = @(q)[l1*cos(q(1)) + l2*cos(q(1)+q(2)) - l3*cos(rho-q(1)); l1*sin(q(1)) + l2*sin(q(1)+q(2)) + l3*sin(rho-q(1))];


syms th1 th2 th3
J = jacobian([l1*cos(th1) + l2*cos(th1+th2) - l3*cos(rho-th1); l1*sin(th1) + l2*sin(th1+th2) + l3*sin(rho-th1)], [th1; th2]);


syms dx dq qi
qi = q;

for i = 1:10
    dx = x_des - fx_ee(qi);
    dq = inv(subs(J, [th1 th2], [qi(1) qi(2)]))*dx;
    qi = eval(qi + dq);
    disp(fx_ee(qi));
end

figure
grid on
hold on
drawLine2D([0,0], fx_1(qi));
drawLine2D(fx_1(qi), fx_2(qi));
drawLine2D(fx_2(qi), fx_ee(qi));

axis equal
