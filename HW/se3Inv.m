function se3Matrix = se3Inv(se)
    % Gets transpose of the rotation matrix
    R = se(1:3,1:3).';
    
    % Multiplies the position vector by the transposed rotation matrix
    P = -R * se(1:3,4);
    se3Matrix = [R P;
                0 0 0 1];
end