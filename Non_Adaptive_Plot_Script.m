%--------------------------------------------------------
% Non-Adaptive Control, Plotting output of Simulink
%--------------------------------------------------------
%% Plotting output (comparison between : reference model, non-adaptive model (or
% we call it exact parameter model), and reference signal)
RGB = [0.9047 0.1918 0.1988;0.2941 0.5447 0.7494;0.3718 0.7176 0.3612;1.0000 0.5482 0.1000;0.8650 0.8110 0.4330;0.6859 0.4035 0.2412];
figure;
hold on;
grid;
plot(Ref_signal.Time, Ref_signal.Data,'-','Color',RGB(1,:),'Linewidth',1);
plot(Ref_signal.Time, Ref_mod_out.Data,'x-','Color',RGB(2,:),'LineWidth',1,'MarkerSize',3);
plot(Ref_signal.Time, Non_adp_out.Data,'--','Color',RGB(6,:),'LineWidth',1,'MarkerSize',1);
hold off;
legend('Ref Signal','Ref Model Output','Non-adaptive Output (Exact)');
title('Comparison of outputs');
xlabel('t(sec)');

%% Plotting of control signal from non-adaptive model (exact parameter model)
figure;
hold on;
grid;
plot(Ref_signal.Time, Non_adp_u.Data,'Color',RGB(1,:));
hold off;
legend('Non adp u');
title('Control Signal of non-adaptive system');
xlabel('t(sec)');