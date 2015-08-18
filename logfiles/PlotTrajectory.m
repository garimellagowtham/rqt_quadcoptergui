function PlotTrajectory( system_params, control_force, cmdtime, indexvrpn, vrpntime, vrpn_matching, constants )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[TrajectoryCost, xs, constants] = EvaluateTrajectory(system_params, control_force, cmdtime, vrpn_matching', constants);

figure(3), clf;
subplot(1,3,1), plot(vrpntime(indexvrpn), vrpn_matching(:,1)),xlabel('time(sec)');
subplot(1,3,2), plot(vrpntime(indexvrpn), vrpn_matching(:,2)),xlabel('time(sec)');
subplot(1,3,3), plot(vrpntime(indexvrpn), vrpn_matching(:,3)),xlabel('time(sec)');

subplot(1,3,1), hold on, plot(cmdtime, xs(1,:),'r'), legend('x','xpred');
subplot(1,3,2), hold on, plot(cmdtime, xs(2,:),'r'), legend('y','ypred');
subplot(1,3,3), hold on, plot(cmdtime, xs(3,:),'r'), legend('z','zpred');

figure(4), clf;
plot(cmdtime(1:end-1), system_params(1)*control_force(5,:)');
hold on, plot(cmdtime, constants.throttle,'r'), legend('Throttlecmd', 'Throttle_out');

disp('Cost: ');
disp(TrajectoryCost);
end

