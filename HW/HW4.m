close all
clc

addpath('.')
addpath('./../matlab_utils')

% hw4arm takes theta_1 through theta_6 in order
% and plots the frames of the arm accordingly

% Problem 1 (a)

% Configure plot
figure
grid on
axis equal
view(45,40)
xlabel('$x$ (m)','interpreter','latex','fontsize',20)
ylabel('$y$ (m)','interpreter','latex','fontsize',20)
zlabel('$z$ (m)','interpreter','latex','fontsize',20)

% Call Function
hw4arm(0, 90*pi/180, 0, 30*pi/180, 90*pi/180, 0);

% Problem 1 (b)

% Configure plot
figure
grid on
axis equal
view(45,40)
xlabel('$x$ (m)','interpreter','latex','fontsize',20)
ylabel('$y$ (m)','interpreter','latex','fontsize',20)
zlabel('$z$ (m)','interpreter','latex','fontsize',20)

% Call Function
hw4arm(0, 120*pi/180, 0, 60*pi/180, 90*pi/180, 0);

% Problem 2 (a)

% Configure plot
figure
grid on
axis equal
view(45,40)
xlabel('$x$ (m)','interpreter','latex','fontsize',20)
ylabel('$y$ (m)','interpreter','latex','fontsize',20)
zlabel('$z$ (m)','interpreter','latex','fontsize',20)

disp('Problem 2(a)')
disp('q = (0, 90, 90, 30, 90, 0)')

% Call Function
hw4arm(0, 90*pi/180, 90*pi/180, 30*pi/180, 90*pi/180, 0);

% Problem 2 (b)

% Configure plot
figure
grid on
axis equal
view(45,40)
xlabel('$x$ (m)','interpreter','latex','fontsize',20)
ylabel('$y$ (m)','interpreter','latex','fontsize',20)
zlabel('$z$ (m)','interpreter','latex','fontsize',20)

disp('Problem 2(b)')
disp('q = (0, 60, 45, 60, 90, 0)')

% Call Function
hw4arm(0, 60*pi/180, 45*pi/180, 60*pi/180, 90*pi/180, 0);




