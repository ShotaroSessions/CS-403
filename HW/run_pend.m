close all
clc
clear

addpath('.')
addpath('./../matlab_utils')

tau1 = 0;
tau2 = 0;
th1 = 3;
th2 = 0;
dth1 = 0;
dth2 = 0;
m1 = 1;
m2 = 1;
I1 = 0.05;
I2 = 0.05;
l1 = 1;
l2 = 0.5;
c1 = 0.5;
c2 = 0.25;
k1 = 10;
k2 = 10;
k3 = 50;
th1_0 = 0;
th2_0 = 0;
l_0 = 0;


g = 9.8;

q = [th1; th2];         % generalized coords
dq = [dth1; dth2];      % 1st d/dt
u = [tau1; tau2];       % controls

z = [q; dq];

% parameters
p = [m1; m2; I1; I2; l1; l2; c1; c2; g];

% Spring attachment point
r0 = [0; 0.5];

% Spring parameters
p = [m1; m2; I1; I2; l1; l2; c1; c2; k1; k2; k3; th1_0; th2_0; l_0; r0; g];

list_t = [];
list_th1 = [];
list_th2 = [];
list_E = [];

for i = 0:700
    t = 0.01 * i;
    dt = 0.01;

    A = A_pend(z, p);
    b = b_pend(z, u, p); % Normal b
    b = b_pend(z, [-z(3); -z(4)], p); % b for part 4
    ddq2 = A\b;

    total_energy = total_e(z, u, p); % Normal total energy 
    total_energy = total_e(z, [-z(3); -z(4)], p); % total energy for part 4

    % z = z + dt*[z(3:4); ddq2]; % Normal Euler Integration
    z = z + dt*[z(3:4) + dt*ddq2; ddq2]; % Semi-Implicit E. I.

    keypoints = get_keypoints(z, p);
    rB = keypoints(1:3);
    rC = keypoints(4:6);


    list_t = [list_t; t];
    list_th1 = [list_th1; z(1)];
    list_th2 = [list_th2; z(2)];
    list_E = [list_E; total_energy];

    clf

    hold on
    drawLine2D([0; 0], rB);
    drawLine2D(rB, rC);

    % Draw Spring
    drawLine2D(r0, rC);


    % axis label
    xlabel('$i\  (m)$','interpreter','latex','fontsize',15)
    ylabel('$j\  (m)$','interpreter','latex','fontsize',15)
    axis equal

    axis([-2 2 -2 2])
    grid on
    hold off

    pause(0.01);
end

figure
plot(list_t, list_th1);
grid on
xlim([0,7]);
xlabel('$t\  (s)$','interpreter','latex','fontsize',15)
ylabel('$\theta_1\  (rad)$','interpreter','latex','fontsize',15)

figure
plot(list_t, list_th2);
grid on
xlim([0,7]);
xlabel('$t\  (s)$','interpreter','latex','fontsize',15)
ylabel('$\theta_2\  (rad)$','interpreter','latex','fontsize',15)

figure
plot(list_t, list_E);
grid on
xlim([0,7]);
xlabel('$t\  (s)$','interpreter','latex','fontsize',15)
ylabel('$Total\ Energy\  (J)$','interpreter','latex','fontsize',15)