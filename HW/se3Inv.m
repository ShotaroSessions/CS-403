function SE3Matrix = SE3Inv(SE)
    % Gets transpose of the rotation matrix
    R = SE(1:3,1:3).';
    
    % Multiplies the position vector by the transposed rotation matrix
    P = -R * SE(1:3,4);
    SE3Matrix = [R P;
                0 0 0 1];
end