function [] = test_consistancy(neesSum, nisSum, tvec, N, alpha )
% This function runs the NIS dynamic consistency test


n = 4;
p = 2;

r1=chi2inv(alpha/2,N*n)./N;
r2=chi2inv(1 - alpha/2,N*n)./N;

r1_NIS = chi2inv(alpha/2,N*p)./N;
r2_NIS = chi2inv(1- alpha/2,N*p)./N;

% Plotting NIS
figure;
subplot(2,2,1)
hold on
scatter(tvec,neesSum,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NEES','fontsize',24,'Interpreter','latex')
title('NEES Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;
subplot(2,2,2)
hold on
scatter(tvec,nisSum,'k','LineWidth',2), hold on
plot(tvec,r1_NIS*ones(1,length(tvec)),'m-')
plot(tvec,r2_NIS*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NIS','fontsize',24,'Interpreter','latex')
title('NIS Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;
subplot(2,2,3)
hold on
scatter(tvec,neesSum,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NEES','fontsize',24,'Interpreter','latex')
ylim([0 30])
title('NEES Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;
subplot(2,2,4)
hold on
scatter(tvec,nisSum,'k','LineWidth',2), hold on
plot(tvec,r1_NIS*ones(1,length(tvec)),'m-')
plot(tvec,r2_NIS*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NIS','fontsize',24,'Interpreter','latex')
ylim([0 30])
title('NIS Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;
set(gcf,'Position', [150 150 750 650]);



end