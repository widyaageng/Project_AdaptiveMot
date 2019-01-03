%--------------------------------------------------------
% Adaptive Control, where Plant parameter are known
%--------------------------------------------------------
clear;clc;

%% Known parameter of True Plant
True_plant_Zp = [1 2];
True_plant_Kp = -0.5;
True_plant_Rp = [1 0.4 4.3];
True_plant = tf([True_plant_Kp*True_plant_Zp],[True_plant_Rp]);

%% Reference Model
time_const = 0.2;
Ref_plant_Rm = [1 1/time_const];
Ref_plant_Km = 1/time_const;
Ref_plant = tf(Ref_plant_Km,[Ref_plant_Rm]);
%% Guessed Plant to get initial value of adaptive parameter
Nat_freq = 2;
Damp_rat = 0.1;
Guessed_plant_Kp = -0.3*(Nat_freq^2);
Guessed_plant_Zp = [1 1];
Guessed_plant_Rp = [1 2*Damp_rat*Nat_freq Nat_freq^2];
Guessed_plant = tf(Guessed_plant_Zp*(Guessed_plant_Kp),Guessed_plant_Rp);
tsim = (0:0.01:100);
usim = [0 ones(1,size(tsim,2)-1)*10]';
figure;
hold on;
lsim(Guessed_plant,usim,tsim);
grid;
axis([0 50 -9 2]);
hold off;
roots(Guessed_plant_Rp)

%% Diophantine Solution & Adaptive Gains
% Choosing Parameter for Non Minimal filter T
push_root = 100; %to set the roots of T, multiple away from plant's poles
dampw = push_root*Nat_freq;
dampT = 0.1;
t1 = 2*dampT*dampw;
t2 = dampw^2;
T_filt = [1 t1 t2];
roots(T_filt)

% Finding polynom E,F,F_bar,G_bar,G1,and Kr of guessed plant
E = [1 1/time_const + t1 - Guessed_plant_Rp(2)];
F = [t2 + (1/time_const)*t1 - Guessed_plant_Rp(3) - Guessed_plant_Rp(2)*E(2) (1/time_const)*t2 - Guessed_plant_Rp(3)*E(2)];
G_bar = conv(E,Guessed_plant_Zp);
F_bar = F/Guessed_plant_Kp;
G1 = G_bar - T_filt;
G1 = G1(2:3);

% Initial Adaptive Parameter
Kr = Ref_plant_Km/Guessed_plant_Kp;
Theta_init_bar = [(-1)*fliplr(F_bar) (-1)*fliplr(G1) Kr];

%% State Space of Non Minimal filter T & Gamma Setting
% Input Filter State Space, Filter output = [wu, swu]
AT_filt_U = [0 1;-t2 -t1];
BT_filt_U = [0;1];
CT_filt_U = [1 0; 0 1];
DT_filt_U = [0;0];

% Output Filter State Space, Filter output = [wy, swy]
AT_filt_Y = [0 1;-t2 -t1];
BT_filt_Y = [0;1];
CT_filt_Y = [1 0; 0 1];
DT_filt_Y = [0;0];

% Gamma setting
Gamma = 10*[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];

%% Plant's dynamic through non-minimal states
nm_A = [0 1 0 0;
    (-1*Guessed_plant_Rp(3)) (-1*Guessed_plant_Rp(1)) Guessed_plant_Zp(2)*Guessed_plant_Kp Guessed_plant_Zp(1)*Guessed_plant_Kp;
    0 0 0 1;
    0 0 -1*T_filt(3) -1*T_filt(2)];
nm_B = [0 0 0 1]';
nm_C = [T_filt(3)-Guessed_plant_Rp(3) T_filt(2)-Guessed_plant_Rp(2) Guessed_plant_Zp(2)*Guessed_plant_Kp Guessed_plant_Zp(1)*Guessed_plant_Kp]';
syms s;
non_minimal_tf = (nm_C'/(s*eye(4)-nm_A))*nm_B;
[nonum, noden] = numden(non_minimal_tf);

%% Dynamic of Adaptive Parameter (State Transition Matrices)

Atheta = zeros(5,5);
Btheta = -1*sign(Guessed_plant_Kp)*Gamma;
Ctheta = eye(5);
Dtheta = zeros(5,5);

%% Getting exact adaptive parameter for comparison purposes

T_filt_exact = [1 t1 t2];
E_exact = [1 1/time_const + t1 - True_plant_Rp(2)];
F_exact = [t2 + (1/time_const)*t1 - True_plant_Rp(3) - True_plant_Rp(2)*E(2) (1/time_const)*t2 - True_plant_Rp(3)*E(2)];

% Theta star vector ,ie : exact [-f2,-f1,-g2,-g1]
G_bar_exact = conv(E_exact,True_plant_Zp);
F_bar_exact = F/True_plant_Kp;
G1_exact = G_bar_exact - T_filt_exact;
G1_exact = G1_exact(2:3);

% Non-adaptive reference gain
Kstar_R_exact = Ref_plant_Km/True_plant_Kp;

% Theta bar star
Theta_star_bar = [(-1)*fliplr(F_bar_exact) (-1)*fliplr(G1_exact) Kstar_R_exact];

