% System parameters
Rm = 1.7897;
Lm = 8.2306e-05;
Bm = 7.4751e-06;
Jm = 1.0942e-05;
Ke = 0.0367;
Kt = 0.01432401;

% State-Space Matrices
A = [0, 1, 0, 0;
     0, -(Bm/Jm), -(1/Jm), (Kt/Jm);
     0, 0, 0, 0;
     0, -(Ke/Lm), 0, -(Rm/Lm)]

B = [0; 0; 0; 1/Lm];

G = [0; 0; 1; 0];

H = [1, 0, 0, 0];  % Measurement matrix, assuming we're measuring the first state (e.g., position)

% Assume zero output matrix for no direct feedthrough
D = [0];

% Discretize the system with sample time of 1 ms
sys = ss(A, B, H, D);  % Create the continuous-time state-space system
sys_d = c2d(sys, 1/1000, 'zoh');  % Discretize the system with sample time of 1 ms

% Extract the discrete-time matrices
A_dis = sys_d.A;
B_dis = sys_d.B;
H_dis = sys_d.C;
D_dis = sys_d.D;

% Display the discretized matrices
disp('Discrete-time A matrix:');
disp(A_dis);

disp('Discrete-time B matrix:');
disp(B_dis);

disp('Discrete-time H matrix:');
disp(H_dis);

disp('Discrete-time D matrix:');
disp(D_dis);
