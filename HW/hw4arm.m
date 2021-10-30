function x = hw4arm(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
    % First build SE(3) matrices for each frame relative
    % to the frame before it
    % frame{1} -> frame{0}
    T01 = SE3(euler2orientation(theta_1, 0, 0), [0; 0; 0]);
    % frame{2} -> frame{1}
    T12 = SE3(euler2orientation(0, theta_2, 0), [0; 0; 0.15]);
    % frame{3} -> frame{2}
    T23 = SE3(euler2orientation(0, theta_3, 0), [0.3; 0; 0]);
    % frame{4} -> frame{3}
    T34 = SE3(euler2orientation(0, 0, theta_4), [0.15; 0; 0]);
    % frame{5} -> frame{4}
    T45 = SE3(euler2orientation(0, theta_5, 0), [0.1; 0; 0]);
    % frame{6} -> frame{5}
    T56 = SE3(euler2orientation(0, 0, theta_6), [0.07; 0; 0]);
    % frame{EE} -> frame{6}
    T6EE = SE3(eye(3), [0.05; 0; 0]);

    % Next get SE(3) of each frame relative to global frame{0}
    T02 = T01*T12; % frame{2} -> frame{0}
    T03 = T02*T23; % frame{3} -> frame{0}
    T04 = T03*T34; % frame{4} -> frame{0}
    T05 = T04*T45; % frame{5} -> frame{0}
    T06 = T05*T56; % frame{6} -> frame{0}
    T0EE = T06*T6EE; % frame{EE} -> frame{0}

    % Draw Coordinates for each fram using T from frame to global
    drawCoordinate3DScale(eye(3), [0;0;0], 0.025);
    drawCoordinate3DScale(T01(1:3,1:3), T01(1:3,4), 0.05);
    drawCoordinate3DScale(T02(1:3,1:3), T02(1:3,4), 0.05);
    drawCoordinate3DScale(T03(1:3,1:3), T03(1:3,4), 0.05);
    drawCoordinate3DScale(T04(1:3,1:3), T04(1:3,4), 0.05);
    drawCoordinate3DScale(T05(1:3,1:3), T05(1:3,4), 0.05);
    drawCoordinate3DScale(T06(1:3,1:3), T06(1:3,4), 0.05);
    drawCoordinate3DScale(T0EE(1:3,1:3), T0EE(1:3,4), 0.1);

    % Draw Lines from each frame's position in global to the next
    drawLine3D(T01(1:3,4), T02(1:3,4))
    drawLine3D(T02(1:3,4), T03(1:3,4))
    drawLine3D(T03(1:3,4), T04(1:3,4))
    drawLine3D(T04(1:3,4), T05(1:3,4))
    drawLine3D(T05(1:3,4), T06(1:3,4))
    drawLine3D(T06(1:3,4), T0EE(1:3,4))

    % Display SE(3): frame{EE} -> frame{0}
    disp("frame{EE} -> frame{0}:")
    disp(T0EE);
end