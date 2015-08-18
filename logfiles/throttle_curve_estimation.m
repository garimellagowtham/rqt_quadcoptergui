%% System ID to find thrust curve for the quadcopter
% Select Folder
%cd session07_August_2015_08_41_57_PM/
%cd session07_August_2015_08_54_32_PM/
%cd session04_August_2015_02_11_06_PM/
%cd session11_August_2015_02_00_43_PM/
%cd session11_August_2015_03_50_28_PM/
%cd session13_August_2015_07_31_00_PM/
%cd session14_August_2015_02_54_55_PM/
cd session17_August_2015_06_29_47_PM/

RC_MIN = [1005,1003,1003,1003];
RC_MAX = [1981,1995,1998,1999];
RC_TRIM = [1513, 1514, 1031, 1510];

cmd = importdata('cmd.dat','\t',1);
ctrlr = importdata('ctrl.dat','\t',1);

vrpn = importdata('vrpn.dat','\t',1);
vrpntime = (vrpn.data(:,1) - vrpn.data(1,1))/1e9;
vrpnrpy = quat2rpy([vrpn.data(:,8) vrpn.data(:,5:7)]);
cd ..

cmdtime = (cmd.data(:,1) - vrpn.data(1,1))/1e9;
throtcmd = (1/(RC_MAX(3) - RC_MIN(3)))*(cmd.data(:,5)-RC_MIN(3));

idx = matchnearest(cmdtime, vrpntime);
match_orientation = vrpn.data(idx,5:8);
control_force = zeros(5, length(cmdtime));
control_force(1:4,:) = match_orientation';
control_force(5,:) = throtcmd';
controlforce = zeros(3,length(cmdtime));
for i = 1:length(cmdtime)
    mat = quat2mat([match_orientation(i,4); match_orientation(i,1); match_orientation(i,2); match_orientation(i,3)]);
    controlforce(:,i) = mat(:,3)*throtcmd(i);%3rd column
end

%% Plot the data in between two times:
t0 = 35;
t1 = 47;
%t0 = 70;
%t1 = 83;
index1 = matchnearest(t0, cmdtime);%For commands
index2 = matchnearest(t1, cmdtime);
indexvrpn = matchnearest(cmdtime(index1:index2), vrpntime);
vrpn_matching = vrpn.data(indexvrpn,2:4);
%%
figure(1), clf; 
subplot(2,3,1), plot(vrpntime(indexvrpn), vrpn_matching(:,1)),xlabel('time(sec)'), legend('x');
subplot(2,3,2), plot(vrpntime(indexvrpn), vrpn_matching(:,2)),xlabel('time(sec)'), legend('y');
subplot(2,3,3), plot(vrpntime(indexvrpn), vrpn_matching(:,3)),xlabel('time(sec)'), legend('z');

subplot(2,3,4), plot(cmdtime(index1:index2), controlforce(1,index1:index2)),xlabel('time(sec)'), legend('forcex');
subplot(2,3,5), plot(cmdtime(index1:index2), controlforce(2,index1:index2)),xlabel('time(sec)'), legend('forcey');
subplot(2,3,6), plot(cmdtime(index1:index2), controlforce(3,index1:index2)),xlabel('time(sec)'), legend('forcez');

figure(2), clf;
subplot(2,2,1), plot(vrpntime(indexvrpn),vrpnrpy(indexvrpn,1)*(180/pi)), xlabel('time(sec)'), ylabel('roll(t)^o');
subplot(2,2,2), plot(vrpntime(indexvrpn),vrpnrpy(indexvrpn,2)*(180/pi)), xlabel('time(sec)'), ylabel('pitch(t)^o');
subplot(2,1,2), plot(vrpntime(indexvrpn),vrpnrpy(indexvrpn,3)*(180/pi)), xlabel('time(sec)'), ylabel('yaw(t)^o');
%% Plot Integrated Trajectories vs Actual Measurements:
%system_params = zeros(5,1);
 system_params = [     29.5348
    0.0160
    0.0035
   -0.0028
    0.0006
    0.0502];
% system_params(1) = 17.9808;%Gain
% system_params(2) = 0.2644;
% system_params(3) = 0.0989;
% %Setup Constants for the system:
constants.R = eye(3);
constants.P = zeros(6);
constants.system_params_mean = system_params;
constants.mass = 1.7;
constants.z_offset = -9.81;%Acceleration
PlotTrajectory( system_params, control_force(:,index1:index2-1), cmdtime(index1:index2), indexvrpn, vrpntime, vrpn_matching, constants )
%% Try setting up fminunc:
options = optimoptions('fminunc','TolX',1e-8);
% system_params(1) = 28;%Gain
% system_params(2) = 0;
% system_params(3) = 0;
% system_params(4) = 0;   
% system_params(5) = 0;
% system_params(6) = 0;
constants.system_params_mean = system_params;
%constants.R(3,3) = 1;
constants.R = 100*eye(3);
constants.P(2,2) = 1e3;
constants.P(3,3) = 1e3;
constants.P(4,4) = 1e3;
constants.P(5,5) = 1e3;
constants.P(6,6) = 1e3;
% constants.P(3,3) = 100;
% constants.P(4,4) = 100;
% constants.P(5,5) = 1e7;
% constants.P(6,6) = 1e7;
[X, Fval, Exitflag, OUTPUT] = fminunc(@(u)EvaluateTrajectory(u, control_force(:,index1:index2-1), cmdtime(index1:index2), vrpn_matching', constants),system_params, options);
PlotTrajectory( X, control_force(:,index1:index2-1), cmdtime(index1:index2), indexvrpn, vrpntime, vrpn_matching, constants )
disp('Final System Params:');
disp(X);



    