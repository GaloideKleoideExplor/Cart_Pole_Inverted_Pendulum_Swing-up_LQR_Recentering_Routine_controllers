%% CART-POLE LQR FOR HEBER'S HARDWARE
% Physical parameters
M = 0.21;        % cart mass [kg]
m = 0.044;       % pole mass [kg]
l = 0.175;       % pole COM distance [m]
g = 9.81;        % gravity [m/s^2]

%% State vector:
% x = [ x; x_dot; theta; theta_dot ]
% theta = 0 means upright

%% Linearized dynamics around upright
A = [ 0      1                0                     0;
    0      0        (m*g)/M                     0;
    0      0                0                     1;
    0      0   ((M+m)*g)/(M*l)                   0 ];

B = [    0;
    1/M;
    0;
    1/(M*l) ];

%% LQR weights tuned for weak actuator
Q = diag([ 5, 0.5, 200, 5 ]);   % [x, x_dot, theta, theta_dot]
R = 0.5;

%% Compute LQR gain
K = lqr(A, B, Q, R);

%% Display results
disp('LQR Gain K = [Kx  Kxdot  Ktheta  Kthetad]');
disp(K);
