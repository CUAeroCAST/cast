function [  ] = test_NEES( neesSum, tvec, N, alpha )
% This function runs the NEES dynamic consistency test

n=length(tvec);

r1=chi2inv(alpha/2,N*n)./N;
r2=chi2inv(1-(alpha/2),N*n)./N;

% Plotting NEES
figure;
hold on
scatter(tvec,neesSum,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('NEES','fontsize',24,'Interpreter','latex')
title('NEES Dynamic Consistency Test')
set(gca, 'FontSize', 20)
grid minor;


end