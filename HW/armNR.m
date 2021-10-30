
close all
clc

addpath('.')
addpath('./../matlab_utils')


syms theta1 theta2 theta3 theta4 theta5 theta6
x_des = [0.1; 0.5; 0.2];
q = [0; 0; 0; 0; 0; 0;];

% frame{1} -> frame{0}
T_01 = SE3([cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1], [0; 0; 0]);

% frame{2} -> frame{1}
T_12 = SE3([cos(theta2) 0 sin(theta2); 0 1 0; -sin(theta2) 0 cos(theta2)], [0; 0; 0.15]);

% frame{3} -> frame{2}
T_23 = SE3([cos(theta3) 0 sin(theta3); 0 1 0; -sin(theta3) 0 cos(theta3)], [0.3; 0; 0]);

% frame{4} -> frame{3}
T_34 = SE3([1 0 0; 0 cos(theta4) -sin(theta4); 0 sin(theta4) cos(theta4)], [0.15; 0; 0]);

% frame{5} -> frame{4}
T_45 = SE3([cos(theta5) 0 sin(theta5); 0 1 0; -sin(theta5) 0 cos(theta5)], [0.1; 0; 0]);

% frame{6} -> frame{5}
T_56 = SE3([1 0 0; 0 cos(theta6) -sin(theta6); 0 sin(theta6) cos(theta6)], [0.07; 0; 0]);

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

syms xi dx dq qi distance
qi = q;

x = T_0EE(1:3, 4);
J = jacobian(x, [theta1 theta2 theta3 theta4 theta5 theta6]);
Jinv = J'*(J*J');

xi = subs(T_0EE(1:3, 4), [theta1 theta2 theta3 theta4 theta5 theta6], [qi(1) qi(2) qi(3) qi(4) qi(5) qi(6)]);
distance = norm(x_des - xi);

for i = 1:20
    xi = subs(T_0EE(1:3, 4), [theta1 theta2 theta3 theta4 theta5 theta6], [qi(1) qi(2) qi(3) qi(4) qi(5) qi(6)]);
    
    dx = x_des - xi;
    dq = subs(Jinv, [theta1 theta2 theta3 theta4 theta5 theta6], [qi(1) qi(2) qi(3) qi(4) qi(5) qi(6)])*dx;
    qi = eval(qi + dq);
    
    distance = eval(norm(x_des - xi));
    
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
    pause(0.01);
    hold off

    disp("Distance between EE and x_des:");
    disp(distance);
end