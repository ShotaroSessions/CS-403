function rMatrix = euler2orientation(theta1, theta2, theta3)
    % Euler ZYX
    % Function returning orientation matrix for yaw rotation of theta
    yaw = @(theta) [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

    % Function returning orientation matrix for pitch rotation of theta
    pitch = @(theta) [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

    % Function returning orientation matrix for roll rotation of theta
    roll = @(theta) [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];

    % Multiply Yaw Pitch Roll rotations matrices to get combined orientation
    % matrix
    rMatrix = yaw(theta1) * pitch(theta2) * roll(theta3);
end