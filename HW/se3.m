function se3Matrix = se3(R,P)
    % SE(3) =  R P
    %          0 1
    %
    se3Matrix = [R P;
                0 0 0 1];
end