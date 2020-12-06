function [  ] = test_NEES( xktrue, xk_plus, Pk_plus, tvec, N, n,alpha )
% This function runs the NEES dynamic consistency test

for i=1:length(tvec)
    exk(:,i) = xktrue(:,i) - xk_plus(:,i);
    NEES(i) = exk(:,i)' * inv(Pk_plus{i}) * exk(:,i);
end

r1=chi2inv(alpha/2,N*n)./N;
r2=chi2inv(1-(alpha/2),N*n)./N;

% Plotting NEES
figure;
hold on
scatter(tvec,NEES,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('$\epsilon_{x,k}$','fontsize',24,'Interpreter','latex')
title(sprintf('NEES Dynamic Consistency Test for alpha = %d and N= %d',alpha,N),'fontsize',12)
set(gca, 'FontSize', 20)
grid minor;


end