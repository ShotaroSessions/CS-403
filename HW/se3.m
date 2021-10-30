function SE3Matrix = SE3(R,P)
    % SE(3) =  R P
    %          0 1
    %
    SE3Matrix = [R P;
                0 0 0 1];
end