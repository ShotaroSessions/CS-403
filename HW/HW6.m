close all
clc

addpath('.')
addpath('./../matlab_utils')

syms t th1 th2 dth dth1 dth2 ddth1 ddth2 m1 m2 I1 I2 l1 l2 c1 c2 g...
     k1 k2 k3 th1_0 th2_0 l_0 rx ry...
     tau1 tau2 T F tau real


q = [th1; th2];         % generalized coords
dq = [dth1; dth2];      % 1st d/dt
ddq = [ddth1; ddth2];   % 2nd d/dt
u = [tau1; tau2];           % controls

% Spring attachment point
r0 = [rx; ry];


% parameters
p = [m1; m2; I1; I2; l1; l2; c1; c2; g];

% Spring parameters
p = [m1; m2; I1; I2; l1; l2; c1; c2; k1; k2; k3; th1_0; th2_0; l_0; r0; g];


% Function to take derivatives
ddt = @(r) jacobian(r, [q; dq])*[dq;ddq];

% Unit vectors
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat, jhat);

% Unit vector in the direction of link 1
Ahat = ihat*sin(th1) - jhat*cos(th1);
% Unit vector in the direction of link 2
Bhat = ihat*sin(th1+th2) - jhat*cos(th1+th2);

% Vector to COM of link 1
rCM1 = c1*Ahat;
% Vector to point B on the HW diagram
rB = l1*Ahat;
% Vector to COM of link 2
rCM2 = rB + c2*Bhat;
% Vector to point C on the HW diagram
rC = rB + l2*Bhat;

% Take derivatives
drCM1 = ddt(rCM1);
drB = ddt(rB);
drCM2 = ddt(rCM2);
drC = ddt(rC);

% Kinetic Energy
T1 = (1/2)*m1*dot(drCM1, drCM1) + (1/2) * I1 * (dth1)^2;
T2 = (1/2)*m2*dot(drCM2, drCM2) + (1/2) * I2 * (dth1 + dth2)^2;

% Gravitational Potential Energy
Vg1 = m1*g*dot(rCM1, -(-jhat));
Vg2 = m2*g*dot(rCM2, -(-jhat));

% Springs Potential Energy
Ve1 = (1/2)*k1*(th1-th1_0)^2;
Ve2 = (1/2)*k2*(th2-th2_0)^2;
Ve3 = (1/2)*k3*(norm(rC-[r0; 0])-l_0)^2;

% Normal Lagrangian
L = T1+T2-Vg1-Vg2;

% Springs Lagrangian
L = T1+T2-Vg1-Vg2-Ve1-Ve2-Ve3;

% Moment to Generalized Force Function
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));

% Convert tau1 and tau2 to Generalized Forces
Qt1 = M2Q(tau1*khat, dth1*khat);
Qt2 = M2Q(tau2*khat, (dth1+dth2)*khat);
Qt3 = M2Q(tau2*khat, (-dth1)*khat);
Q = Qt1 + Qt2 + Qt3;

% Implicit Form
g = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;

% Get q double dot
A = jacobian(g,ddq);
b = A*ddq - g;
ddq2 = A\b;

% Form z and dz
z = [q; dq];
dz = [dq; ddq2];

% Normal Total Energy
E = T1 + T2 + Vg1 + Vg2;

% Spring Total Energy
E = T1 + T2 + Vg1 + Vg2 + Ve1 +Ve2 +Ve3;

keypoints = [rB; rC];

matlabFunction(A, 'file', ['A_pend'], 'vars', {z p});
matlabFunction(b, 'file', ['b_pend'], 'vars', {z u p});
matlabFunction(keypoints, 'file', ['get_keypoints'], 'vars', {z p});
matlabFunction(E, 'file', ['total_e'], 'vars', {z u p});




      