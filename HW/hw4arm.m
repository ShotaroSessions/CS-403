function x = hw4arm(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
    % First build SE(3) matrices for each frame relative
    % to the frame before it

    % frame{1} -> frame{0}
    T_01 = SE3(euler2orientation(theta_1, 0, 0), [0; 0; 0]);

    % frame{2} -> frame{1}
    T_12 = SE3(euler2orientation(0, theta_2, 0), [0; 0; 0.15]);

    % frame{3} -> frame{2}
    T_23 = SE3(euler2orientation(0, theta_3, 0), [0.3; 0; 0]);

    % frame{4} -> frame{3}
    T_34 = SE3(euler2orientation(0, 0, theta_4), [0.15; 0; 0]);

    % frame{5} -> frame{4}
    T_45 = SE3(euler2orientation(0, theta_5, 0), [0.1; 0; 0]);

    % frame{6} -> frame{5}
    T_56 = SE3(euler2orientation(0, 0, theta_6), [0.07; 0; 0]);

    % frame{EE} -> frame{6}
    T_6EE = SE3(eye(3), [0.05; 0; 0]);

    % Next get SE(3) of each frame relative to global frame{0}

    T_02 = T_01*T_12; % frame{2} -> frame{0}
    T_03 = T_02*T_23; % frame{3} -> frame{0}
    T_04 = T_03*T_34; % frame{4} -> frame{0}
    T_05 = T_04*T_45; % frame{5} -> frame{0}
    T_06 = T_05*T_56; % frame{6} -> frame{0}
    T_0EE = T_06*T_6EE; % frame{EE} -> frame{0}

    % Draw Coordinates for each fram using T from frame to global
    drawCoordinate3DScale(eye(3), [0;0;0], 0.025);
    drawCoordinate3DScale(T_01(1:3,1:3), T_01(1:3,4), 0.05);
    drawCoordinate3DScale(T_02(1:3,1:3), T_02(1:3,4), 0.05);
    drawCoordinate3DScale(T_03(1:3,1:3), T_03(1:3,4), 0.05);
    drawCoordinate3DScale(T_04(1:3,1:3), T_04(1:3,4), 0.05);
    drawCoordinate3DScale(T_05(1:3,1:3), T_05(1:3,4), 0.05);
    drawCoordinate3DScale(T_06(1:3,1:3), T_06(1:3,4), 0.05);
    drawCoordinate3DScale(T_0EE(1:3,1:3), T_0EE(1:3,4), 0.1);

    % Draw Lines from each frame's position in global to the next
    drawLine3D(T_01(1:3,4), T_02(1:3,4))
    drawLine3D(T_02(1:3,4), T_03(1:3,4))
    drawLine3D(T_03(1:3,4), T_04(1:3,4))
    drawLine3D(T_04(1:3,4), T_05(1:3,4))
    drawLine3D(T_05(1:3,4), T_06(1:3,4))
    drawLine3D(T_06(1:3,4), T_0EE(1:3,4))

    % Display SE(3): frame{EE} -> frame{0}
    disp("frame{EE} -> frame{0}:")
    disp(T_0EE);
end