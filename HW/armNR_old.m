
close all
clc

addpath('.')
addpath('./../matlab_utils')


syms qi
x_des = [0.5; 0.6; 0.2];
qi = [0; 0; 0; 0; 0; 0;];

% frame{1} -> frame{0}
T_01 = SE3([cos(qi(1)) -sin(qi(1)) 0; sin(qi(1)) cos(qi(1)) 0; 0 0 1], [0; 0; 0]);

% frame{2} -> frame{1}
T_12 = SE3([cos(qi(2)) 0 sin(qi(2)); 0 1 0; -sin(qi(2)) 0 cos(qi(2))], [0; 0; 0.15]);

% frame{3} -> frame{2}
T_23 = SE3([cos(qi(3)) 0 sin(qi(3)); 0 1 0; -sin(qi(3)) 0 cos(qi(3))], [0.3; 0; 0]);

% frame{4} -> frame{3}
T_34 = SE3([1 0 0; 0 cos(qi(4)) -sin(qi(4)); 0 sin(qi(4)) cos(qi(4))], [0.15; 0; 0]);

% frame{5} -> frame{4}
T_45 = SE3([cos(qi(5)) 0 sin(qi(5)); 0 1 0; -sin(qi(5)) 0 cos(qi(5))], [0.1; 0; 0]);

% frame{6} -> frame{5}
T_56 = SE3([1 0 0; 0 cos(qi(6)) -sin(qi(6)); 0 sin(qi(6)) cos(qi(6))], [0.07; 0; 0]);

% frame{EE} -> frame{6}
T_6EE = SE3(eye(3), [0.05; 0; 0]);

% Next get SE(3) of each frame relative to global frame{0}

T_02 = T_01*T_12; % frame{2} -> frame{0}
T_03 = T_02*T_23; % frame{3} -> frame{0}
T_04 = T_03*T_34; % frame{4} -> frame{0}
T_05 = T_04*T_45; % frame{5} -> frame{0}
T_06 = T_05*T_56; % frame{6} -> frame{0}
T_0EE = T_06*T_6EE; % frame{EE} -> frame{0}


figure

syms dx dq
qi = q;
disp(T_0EE);
x = T_0EE(1:3, 4);

Jinv = pinv(jacobian(x, qi));

for i = 1:10
    dx = x_des - T_0EE(1:3, 4);
    dq = Jinv*dx;
    qi = eval(qi + dq);
end
    hold on
    clf    
    % Configure plot
    grid on
    axis equal
    view(45,40)
    xlabel('$x$ (m)','interpreter','latex','fontsize',20)
    ylabel('$y$ (m)','interpreter','latex','fontsize',20)
    zlabel('$z$ (m)','interpreter','latex','fontsize',20)
    
    % Plot arm
    hw4arm(qi(1), qi(2), qi(3), qi(4), qi(5), qi(6));
    pause(1);
    hold off


% % Draw Coordinates for each fram using T from frame to global
% drawCoordinate3DScale(eye(3), [0;0;0], 0.025);
% drawCoordinate3DScale(T_01(1:3,1:3), T_01(1:3,4), 0.05);
% drawCoordinate3DScale(T_02(1:3,1:3), T_02(1:3,4), 0.05);
% drawCoordinate3DScale(T_03(1:3,1:3), T_03(1:3,4), 0.05);
% drawCoordinate3DScale(T_04(1:3,1:3), T_04(1:3,4), 0.05);
% drawCoordinate3DScale(T_05(1:3,1:3), T_05(1:3,4), 0.05);
% drawCoordinate3DScale(T_06(1:3,1:3), T_06(1:3,4), 0.05);
% drawCoordinate3DScale(T_0EE(1:3,1:3), T_0EE(1:3,4), 0.1);
% 
% % Draw Lines from each frame's position in global to the next
% drawLine3D(T_01(1:3,4), T_02(1:3,4))
% drawLine3D(T_02(1:3,4), T_03(1:3,4))
% drawLine3D(T_03(1:3,4), T_04(1:3,4))
% drawLine3D(T_04(1:3,4), T_05(1:3,4))
% drawLine3D(T_05(1:3,4), T_06(1:3,4))
% drawLine3D(T_06(1:3,4), T_0EE(1:3,4))