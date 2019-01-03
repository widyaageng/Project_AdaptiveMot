%--------------------------------------------------------
% Non-Adaptive Control, where Plant parameter are known
%--------------------------------------------------------
clear;clc;

%% Known parameter of True Plant
True_plant_Zp = [1 2];
True_plant_Kp = -0.5;
True_plant_Rp = [1 0.4 4.3];
True_plant = tf([True_plant_Kp*True_plant_Zp],[True_plant_Rp]);

%% Reference Model
time_const = 0.002;
Ref_plant_Rm = [1 1/time_const];
Ref_plant_Km = 1/time_const;
Ref_plant = tf(Ref_plant_Km,[Ref_plant_Rm]);

%% Diophantine Solution
% Choosing Parameter for Non Minimal filter T
t1 = 80;
t2 = 1600;
T_filt = [1 t1 t2];
Dio_LHS = conv(T_filt,Ref_plant_Rm);
E = [1 Dio_LHS(2) - True_plant_Rp(2)];
F = [t2 + (1/time_const)*t1 - True_plant_Rp(3) - True_plant_Rp(2)*E(2) (1/time_const)*t2 - True_plant_Rp(3)*E(2)];
G_bar = conv(E,True_plant_Zp);
F_bar = -1*F/True_plant_Kp;
G1 = G_bar - T_filt;
G1 = -1*G1(2:3);

%% State Space of Non Minimal filter T
% Input Filter State Space
AT_filt_U = [0 1;-t2 -t1];
BT_filt_U = [0;1];
CT_filt_U = [G1(2) 0; 0 G1(1)];

% Output Filter State Space
AT_filt_Y = [0 1;-t2 -t1];
BT_filt_Y = [0;1];
CT_filt_Y = [F_bar(2) 0; 0 F_bar(1)];

% Non-adaptive reference gain
Kstar_R = Ref_plant_Km/True_plant_Kp;

