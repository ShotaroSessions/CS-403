function so3V = SO3toso3(SO3)
    % Setting theta in 2 sin(theta) to 1
    so3M = (SO3-SO3')/(2*sin(1));
    so3V = [so3M(3,2); so3M(1,3); so3M(2,1)];
end