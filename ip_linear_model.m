function [A_up, B_up, K] = ip_linear_model()

    % ---------------------------------------------------------
    % Physical Parameters
    % ---------------------------------------------------------
    m  = 0.2;
    M  = 0.8;
    L  = 0.75;
    g  = 9.81;

    I  = m * L^2;

    Bv = 0.01;
    C  = 2.0;

    % ---------------------------------------------------------
    % Mass matrix terms at UPRIGHT
    % ---------------------------------------------------------
    A11 = M + m;
    A12 = -m * L;
    A21 = A12;
    A22 = I + m * L^2;
    D   = A11*A22 - A12*A21;

    disp('--- Mass Matrix Terms ---')
    disp(['A11 = ', num2str(A11)])
    disp(['A12 = ', num2str(A12)])
    disp(['A22 = ', num2str(A22)])
    disp(['D   = ', num2str(D)])

    % ---------------------------------------------------------
    % Linearization
    % ---------------------------------------------------------
    A_up = [ 0, 0, 1, 0;
             0, 0, 0, 1;
             0, -(A12)*m*g*L/D,   -A22*C/D,    A12*Bv/D;
             0,  (A11)*m*g*L/D,    A21*C/D,   -A11*Bv/D ];

    B_up = [ 0;
             0;
             A22/D;
            -A21/D ];

    disp(' ')
    disp('--- Linearized A_up ---')
    disp(A_up)

    disp('--- Linearized B_up ---')
    disp(B_up)

    % ---------------------------------------------------------
    % DRIFT-SUPPRESSION LQR v3 (VERY STRONG CENTERING)
    % ---------------------------------------------------------

    % Tight Bryson bounds
    x_max        = 0.14;      % VERY tight → strong centering
    theta_max    = 0.03;
    xdot_max     = 1;
    thetadot_max = 4;

    Q_bryson = diag([ ...
        1/(x_max^2), ...
        1/(theta_max^2), ...
        1/(xdot_max^2), ...
        1/(thetadot_max^2) ...
    ]);

    % Strong but catchable
    q_scale = 1;
    Q = q_scale * Q_bryson;

    % EXTREME drift suppression
    Q(1,1) = Q(1,1) * 1;   % cart position
    Q(3,3) = Q(3,3) * 0;   % cart velocity

    % Slightly stronger angle penalty
    Q(2,2) = Q(2,2) * 1;

    % R tuned for drift suppression + catchability
    R = 0.035;

    disp(' ')
    disp('--- Q Matrix (Drift-Suppressing v3) ---')
    disp(Q)

    disp('--- R Value ---')
    disp(R)

    K = lqr(A_up, B_up, Q, R);

    disp(' ')
    disp('--- LQR Gain K (Drift-Suppressing v3) ---')
    disp(K)
    disp(-K)

end
