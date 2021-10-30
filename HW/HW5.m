close all
clc

addpath('.')
addpath('./../matlab_utils')

syms th J

T_des = [0, -1, 0, 0.2;
         1,  0, 0, 0.31;
         0,  0, 1, 0.2;
         0,  0, 0, 1];

th(1) = 0.01;
th(2) = -0.01;
th(3) =-0.01;
th(4) = -0.01;
th(5) = 0.01;
th(6) = 0.01;

while true    
    % frame{1} -> frame{0}
    T_01 = SE3([cos(th(1)) -sin(th(1)) 0; sin(th(1)) cos(th(1)) 0; 0 0 1], [0; 0; 0]);
    % frame{2} -> frame{1}
    T_12 = SE3([cos(th(2)) 0 sin(th(2)); 0 1 0; -sin(th(2)) 0 cos(th(2))], [0; 0; 0.15]);
    % frame{3} -> frame{2}
    T_23 = SE3([cos(th(3)) 0 sin(th(3)); 0 1 0; -sin(th(3)) 0 cos(th(3))], [0.3; 0; 0]);
    % frame{4} -> frame{3}
    T_34 = SE3([1 0 0; 0 cos(th(4)) -sin(th(4)); 0 sin(th(4)) cos(th(4))], [0.15; 0; 0]);
    % frame{5} -> frame{4}
    T_45 = SE3([cos(th(5)) 0 sin(th(5)); 0 1 0; -sin(th(5)) 0 cos(th(5))], [0.1; 0; 0]);
    % frame{6} -> frame{5}
    T_56 = SE3([1 0 0; 0 cos(th(6)) -sin(th(6)); 0 sin(th(6)) cos(th(6))], [0.07; 0; 0]);
    % frame{EE} -> frame{6}
    T_6EE = SE3(eye(3), [0.05; 0; 0]);

    % frame{EE} -> frame{0}
    T_0EE = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6EE;

    % Get Jacobian
    J = hw5jacob(th);
    
    % Get error
    err = zeros(6,1);
    err(1:3) = SO3toso3(T_des(1:3,1:3)*T_0EE(1:3,1:3)');
    err(4:6) = T_des(1:3,4) - T_0EE(1:3,4);
    
    % Get new theta
    th = th + 0.2*pinv(J)*err;
    
    % Normalize error
    normErr = norm(err);
    
    hold on
    clf    
    % Configure plot
    grid on
    axis equal
    view(45,40)
    xlabel('$x$ (m)','interpreter','latex','fontsize',20)
    ylabel('$y$ (m)','interpreter','latex','fontsize',20)
    zlabel('$z$ (m)','interpreter','latex','fontsize',20)

    % Draw T_des
    drawCoordinate3DScale(T_des(1:3,1:3), T_des(1:3,4), 0.1);
    
    % Plot arm
    hw4arm(th(1), th(2), th(3), th(4), th(5), th(6));
    pause(0.01);
    hold off

    % Display Error
    disp("Error between EE and T_des:");
    disp(normErr);
    
    % Break if error is small enough
    if normErr < 0.001
        break
    end
end