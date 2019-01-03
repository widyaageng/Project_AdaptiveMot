%--------------------------------------------------------
% Adaptive Control, Plotting output of Simulink
%--------------------------------------------------------
clc;
%% Plotting output (comparison between : reference model, non-adaptive model (or
% we call it exact parameter model), and reference signal)
RGB = [0.3467 0.5360 0.6907;0.9153 0.2816 0.2878;0.4416 0.7490 0.4322];
figure;
hold on;
grid;
plot(Ref_signal.Time, Ref_signal.Data,'-','Color','k','Linewidth',0.5);
plot(Adp_out.Time, Adp_out.Data,'-x','Color',RGB(3,:),'LineWidth',1,'MarkerSize',1);
plot(Ref_signal.Time, Ref_mod_out.Data,'-.','Color',RGB(2,:),'LineWidth',1,'MarkerSize',2);
hold off;
legend('Ref Signal','Adaptive Output','Ref Model Output');
title('Comparison of outputs');
xlabel('t(sec)');

%% Plotting of control signal from non-adaptive model (exact parameter model)
figure;
hold on;
grid;
plot(Ref_signal.Time, Adp_u.Data,'Color',RGB(1,:));
hold off;
legend('Non adp u');
title('Control Signal of Adaptive system');
xlabel('t(sec)');

%% Plotting of Adaptive gains and its comparison against exact adaptive gains
figure;

% Plotting theta 1, ie: -f2
subplot(2,1,1);grid;hold on;
plot(Adp_Gain.Time, Adp_Gain.Data(:,1),'-','Color',RGB(2,:),'Linewidth',1);
plot(Adp_Gain.Time, Theta_star_bar(1)*ones(size(Adp_Gain.Data(:,1),1),1),'--','Color',RGB(1,:),'LineWidth',1,'MarkerSize',1);
xlabel('t(sec)');
title('Comparison of Theta 1');
legend('Theta 1','Exact Theta 1');
range = max(max(Adp_Gain.Data(:,1),Theta_star_bar(1))) - min(min(Adp_Gain.Data(:,1),Theta_star_bar(1))) + 0.5;
axis([Adp_Gain.Time(1) Adp_Gain.Time(end) (min(min(Adp_Gain.Data(:,1),Theta_star_bar(1)))-0.1*range) (max(max(Adp_Gain.Data(:,1),Theta_star_bar(1)))+0.1*range)]);
hold off;

% Plotting theta 1, ie: -f1
subplot(2,1,2);grid;hold on;
plot(Adp_Gain.Time, Adp_Gain.Data(:,2),'-','Color',RGB(2,:),'Linewidth',1);
plot(Adp_Gain.Time, Theta_star_bar(2)*ones(size(Adp_Gain.Data(:,2),1),1),'--','Color',RGB(1,:),'LineWidth',1,'MarkerSize',1);
xlabel('t(sec)');
title('Comparison of Theta 2');
legend('Theta 2','Exact Theta 2');
range = max(max(Adp_Gain.Data(:,2),Theta_star_bar(2))) - min(min(Adp_Gain.Data(:,2),Theta_star_bar(2))) + 0.5;
axis([Adp_Gain.Time(1) Adp_Gain.Time(end) min(min(Adp_Gain.Data(:,2),Theta_star_bar(2))) - 0.1*range max(max(Adp_Gain.Data(:,2),Theta_star_bar(2))) + 0.1*range]);
hold off;

figure;

% Plotting theta 3, ie: -g2
subplot(2,1,1);grid;hold on;
plot(Adp_Gain.Time, Adp_Gain.Data(:,3),'-','Color',RGB(2,:),'Linewidth',1);
plot(Adp_Gain.Time, Theta_star_bar(3)*ones(size(Adp_Gain.Data(:,3),1),1),'--','Color',RGB(1,:),'LineWidth',1,'MarkerSize',1);
xlabel('t(sec)');
title('Comparison of Theta 3');
legend('Theta 3','Exact Theta 3');
range = max(max(Adp_Gain.Data(:,3),Theta_star_bar(3))) - min(min(Adp_Gain.Data(:,3),Theta_star_bar(3))) + 0.5;
axis([Adp_Gain.Time(1) Adp_Gain.Time(end) min(min(Adp_Gain.Data(:,3),Theta_star_bar(3))) - 0.1*range max(max(Adp_Gain.Data(:,3),Theta_star_bar(3))) + 0.1*range]);
hold off;

% Plotting theta 4, ie: -g1
subplot(2,1,2);grid;hold on;
plot(Adp_Gain.Time, Adp_Gain.Data(:,4),'-','Color',RGB(2,:),'Linewidth',1);
plot(Adp_Gain.Time, Theta_star_bar(4)*ones(size(Adp_Gain.Data(:,4),1),1),'--','Color',RGB(1,:),'LineWidth',1,'MarkerSize',3);
xlabel('t(sec)');
title('Comparison of Theta 4');
legend('Theta 4','Exact Theta 4');
range = max(max(Adp_Gain.Data(:,4),Theta_star_bar(4))) - min(min(Adp_Gain.Data(:,4),Theta_star_bar(4))) + 0.5;
axis([Adp_Gain.Time(1) Adp_Gain.Time(end) min(min(Adp_Gain.Data(:,4),Theta_star_bar(4))) - 0.1*range max(max(Adp_Gain.Data(:,4),Theta_star_bar(4))) + 0.1*range]);
hold off;

% Plotting theta 5, ie: K
figure;hold on;grid;
plot(Adp_Gain.Time, Adp_Gain.Data(:,5),'-','Color',RGB(2,:),'Linewidth',1);
plot(Adp_Gain.Time, Theta_star_bar(5)*ones(size(Adp_Gain.Data(:,5),1),1),'--','Color',RGB(1,:),'LineWidth',1,'MarkerSize',1);
xlabel('t(sec)');
title('Comparison of Theta 5');
legend('Theta 5','Exact Theta 5');
range = max(max(Adp_Gain.Data(:,5),Theta_star_bar(5))) - min(min(Adp_Gain.Data(:,5),Theta_star_bar(5))) + 0.5;
axis([Adp_Gain.Time(1) Adp_Gain.Time(end) min(min(Adp_Gain.Data(:,5),Theta_star_bar(5))) - 0.1*range max(max(Adp_Gain.Data(:,5),Theta_star_bar(5))) + 0.1*range]);
hold off;

