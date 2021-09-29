close all
clc

addpath('.')
addpath('./../matlab_utils')


%% Problem 1 

% My function do get rotation matrix from euler angles
% See euler2orientation.m for detailed comments.
R1 = euler2orientation(0.3, 0.2, 0.5);
disp("euler2orientation(0.3, 0.2, 0.5):");
disp(R1);

R2 = euler2orientation(0.7, pi, pi/2);
disp("euler2orientation(0.7, pi, pi/2):");
disp(R2);

R3 = euler2orientation(pi/3, 0, 0);
disp("euler2orientation(pi/3, 0, 0):");
disp(R3);


%% Problem 2


% se3(R,P) builds SE(3) matrix from rotation matrix in R and a position
% vector in P. See se3.m for details on my implementation
%
% se3Mult(se1, se2) multiplies two SE(3) matrices and returns the result,
% see se3Mult.m for details.
%
% se3Inv(se) inverts an SE(3) matrix and returns the result.
% See se3Inv.m for details.

% frame{1} -> frame{0}
P1 = [0.4; 0.8; 1.2];
T01 = se3(R1, P1);
disp("T_01: frame{1} -> frame{0}");
disp(T01);

% frame{2} -> frame{1}
P2 = [-0.4; 0.5; 1.0];
T12 = se3(R2, P2);
disp("T_12: frame{2} -> frame{1}");
disp(T12);

% frame{3} -> frame{2}
P3 = [0.5; -0.8; 1.2];
T23 = se3(R3, P3);
disp("T_23: frame{3} -> frame{2}");
disp(T23);

% frame{3} -> frame{0}
% Multiply/Compose transformations from frames 3 to 0
T02 = se3Mult(T01, T12);
T03 = se3Mult(T02, T23);
disp("T_03: frame{3} -> frame{0}");
disp(T03);


%% Problem 3


% draw figure
figure
grid on

% Plot frame{0} (Global Frame)
drawCoordinate3D(eye(3), [0;0;0;]);

% Plot frame{1}
drawCoordinate3DScale(R1, P1, 0.5);

% Plot frame{2} on frame{0} using T_02
drawCoordinate3DScale(T02(1:3,1:3), T02(1:3,4), 0.5);

% Plot frame{3} on frame{0} using T_03
drawCoordinate3DScale(T03(1:3,1:3), T03(1:3,4), 0.5);

% Figure settings
xlabel('$x$','interpreter','latex','fontsize',20)
ylabel('$y$','interpreter','latex','fontsize',20)
zlabel('$z$','interpreter','latex','fontsize',20)
grid on
axis equal
axis([-0.5 1.5 -1 1.5 0 2.5])
view(40,20)


