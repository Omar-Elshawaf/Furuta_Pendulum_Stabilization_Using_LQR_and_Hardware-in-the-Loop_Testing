g = 9.81;
% 1 - Arm
% 2 - Pendulum 

m1 = 0.03217;   % [kg] arm rod mass
m2 = 0.041;     % [kg] pendulum rod + 5g coupler (0.016 + 0.005)
L1 = 0.095;     % [m] arm length  (KEEPING what you pasted)
L2 = 0.08;     % [m] pendulum length (KEEPING what you pasted)
M  = 0.01;       % [kg] tip mass (you said none)


J = 0.007022721987053; %3.5256e-04            %System
kb_p = 4.7940e-04 ;

kb_m = 0.013983279899632 ;
ke = 0.967774441589921;
Re = 4.857292094830934;  %Medido
alpha = J + (M+m1/3+m2)*L1^2;
beta = (M + m2/3)*L2^2;
gamma = (M + m2/2)*L2*L1;
sigma = (M + m2/2)*g*L2;

%% Simulation parameters

initial_state = pi;
Ts = 0.01;
fc=2;
wc = 2*pi*fc;           % cutoff rad/s
a  = exp(-wc*Ts);       % discrete pole
dtDisc = 0.01;
Reference = [0 0 0 0];
Zn = 3;   %Dead Zone

StepX = 10;
distrub = 12;
disturb = distrub*pi/180;

%% Linearization

% A matrix
A = zeros(4,4);
A(1,2) = 1;
A(2,3) = -(sigma*gamma)/(alpha*beta-gamma^2);
A(3,4) = 1;
A(4,3) = (alpha*sigma)/(alpha*beta-gamma^2);

% B matrix
B = zeros(4,2);
B(2,1) = beta/(alpha*beta-gamma^2);
B(2,2) = -gamma/(alpha*beta-gamma^2);
B(4,1) = -gamma/(alpha*beta-gamma^2);
B(4,2) = alpha/(alpha*beta-gamma^2);

% C matrix
C = [0 0 1 0;
     0 0 0 1];

%% Pseudo Linear System

% Pseudo A matrix
Ap = zeros(4,4);

Ap(1,2) = 1;

Ap(2,1) = 0;      Ap(2,2) = -B(2,1)*(ke^2/Re + kb_m); 
Ap(2,3) = A(2,3); Ap(2,4) = -B(2,2)*kb_p;

Ap(3,4) = 1;

Ap(4,1) = 0;      Ap(4,2) = -B(4,1)*(ke^2/Re + kb_m); 
Ap(4,3) = A(4,3); Ap(4,4) = -B(4,2)*kb_p;

% Pseudo B matrix

Bp = zeros(4,1);

Bp(2) = B(2,1)*ke/Re;  
Bp(4) = B(4,1)*ke/Re;   

% Controlability and Observability
Control = rank(ctrb(Ap,Bp));
Observ = rank(obsv(Ap,C));

% State Feedback Control

%K = place(Ap,Bp,[-5 -4 -2+2j -2-2j]);  
%K = place(Ap,Bp,[-8+8j -8-8j  -4+4j -4-4j]);


Q = [60 0 0 0;
     0 250 0 0;
     0 0 750 0;
     0 0 0 50];
R = 1.07;
[K, ~, E] = lqr(Ap,Bp,Q,R);

%K -0,4473 -1,3203 -33,3556 -3,0467
%K -0.2000   -1.1342  -30.9420   -3.2791  - Funciona pendulo 16/01

%% Down position (pi)

% Pseudo A matrix around pi
Ap2 = zeros(4,4);

Ap2(1,2) = 1;

Ap2(2,1) = 0;      Ap2(2,2) = -B(2,1)*(ke^2/Re + kb_m); 
Ap2(2,3) = A(2,3); Ap2(2,4) = B(2,2)*kb_p;

Ap2(3,4) = 1;

Ap2(4,1) = 0;      Ap2(4,2) = B(4,1)*(ke^2/Re + kb_m); 
Ap2(4,3) = -A(4,3); Ap2(4,4) = -B(4,2)*kb_p;

% Pseudo B matrix around pi

Bp2 = zeros(4,1);

Bp2(2) = B(2,1)*ke/Re;  
Bp2(4) = -B(4,1)*ke/Re;  

K2 = place(Ap2,Bp2,[-5 -4 -2+2j -2-2j]);
R2 = 5;
Q2=[1 0 0 0;
    0 10 0 0;
    0 0 600 0;
    0 0 0 50];
[K2, ~, E] = lqr(Ap2,Bp2,Q2,R2);
K2