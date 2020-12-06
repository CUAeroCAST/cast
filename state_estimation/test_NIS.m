function [  ] = test_NIS( yk, yk_minus, Pk_minus,H,R, tvec, N, p, alpha )
% This function runs the NIS dynamic consistency test

for i=1:length(tvec)
    S{i} = H * Pk_minus{i} * H' + R;
    eyk(:,i) = yk(:,i) - yk_minus(:,i);
    NIS(i) = eyk(:,i)' * inv(S{i}) * eyk(:,i);
end

r1=chi2inv(alpha/2,N*p)./N;
r2=chi2inv(1-(alpha/2),N*p)./N;

% Plotting NIS
figure;
hold on
scatter(tvec,NIS,'k','LineWidth',2), hold on
plot(tvec,r1*ones(1,length(tvec)),'m-')
plot(tvec,r2*ones(1,length(tvec)),'m-')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('$\epsilon_{y,k}$','fontsize',24,'Interpreter','latex')
title(sprintf('NIS Dynamic Consistency Test for alpha = %d and N= %d',alpha,N),'fontsize',12)
set(gca, 'FontSize', 20)
grid minor;



end