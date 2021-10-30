function J = hw5jacob(th)
    s1 = [0;0;1;0;0;0];
    s2 = [0;1;0;0;0;0];
    s3 = [0;1;0;0;0;0];
    s4 = [1;0;0;0;0;0];
    s5 = [0;1;0;0;0;0];
    s6 = [1;0;0;0;0;0];

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

    % frame{1} -> frame{EE}
    T_EE1 = SE3Inv(T_12 * T_23 * T_34 * T_45 * T_56 * T_6EE);
    % frame{2} -> frame{EE}
    T_EE2 = SE3Inv(T_23 * T_34 * T_45 * T_56 * T_6EE);
    % frame{3} -> frame{EE}
    T_EE3 = SE3Inv(T_34 * T_45 * T_56 * T_6EE);
    % frame{4} -> frame{EE}
    T_EE4 = SE3Inv(T_45 * T_56 * T_6EE);
    % frame{5} -> frame{EE}
    T_EE5 = SE3Inv(T_56 * T_6EE);
    % frame{6} -> frame{EE}
    T_EE6 = SE3Inv(T_6EE);

    % frame{EE} -> frame{0}
    T_0EE = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6EE;

    % build adjoint matrices for each joint
    Adj_EE1 = Adj(T_EE1);
    Adj_EE2 = Adj(T_EE2);
    Adj_EE3 = Adj(T_EE3);
    Adj_EE4 = Adj(T_EE4);
    Adj_EE5 = Adj(T_EE5);
    Adj_EE6 = Adj(T_EE6);

    % construct Jacobian from adjoint matrices
    J = [Adj_EE1*s1, Adj_EE2*s2, Adj_EE3*s3, Adj_EE4*s4, Adj_EE5*s5, Adj_EE6*s6];
    
    % Jacobian to global
    R_0EE = zeros(6);
    R_0EE(1:3, 1:3) = T_0EE(1:3,1:3);
    R_0EE(4:6, 4:6) = T_0EE(1:3,1:3);
    J = R_0EE*J;
end