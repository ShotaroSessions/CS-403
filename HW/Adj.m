function adj = Adj(T)
    adj = zeros(6);
    adj(1:3,1:3) = T(1:3,1:3);
    skew = [0, -T(3,4), T(2,4);
            T(3,4),  0, -T(1,4);
            -T(2,4), T(1,4), 0]; 
    adj(4:6,1:3) = skew*T(1:3,1:3);
    adj(4:6,4:6) = T(1:3,1:3);
end