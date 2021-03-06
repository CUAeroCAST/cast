function [  ] = test_NIS(nisSum, tvec, N, alpha )
% This function runs the NIS dynamic consistency test


n=length(tvec);

r1=chi2inv(alpha/2,N*n)./N;
r2=chi2inv(1-(alpha/2),N*n)./N;

% Plotting NIS
figure;
hold on
scatter(tvec,nisSum,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NIS','fontsize',24,'Interpreter','latex')
title('NIS Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;



end