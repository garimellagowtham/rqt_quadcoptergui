cd session17_August_2015_02_55_14_PM/

RC_MIN = [1005,1003,1003,1003];
RC_MAX = [1981,1995,1998,1999];
RC_TRIM = [1513, 1514, 1031, 1510];

cmd = importdata('cmd.dat','\t',1);
ctrlr = importdata('ctrl.dat','\t',1);

vrpn = importdata('vrpn.dat','\t',1);
vrpntime = (vrpn.data(:,1) - vrpn.data(1,1))/1e9;
vrpnrpy = quat2rpy([vrpn.data(:,8) vrpn.data(:,5:7)]);

cmdtime = (cmd.data(:,1) - vrpn.data(1,1))/1e9;

cd ..
%% Load Variables from apm log
% Assuming m file is loaded from SDCard and RCIN and ROU struct are in workspace
t_offset = 1.05;
rcintime = (RCIN.data(:,2) - RCIN.data(1,2))/1e6 - t_offset;
rcouttime = (RCOU.data(:,2) - RCIN.data(1,2))/1e6 - t_offset;

%% Plot RC Data from apm log
figure; plot(cmdtime, cmd.data(:,2));
hold on; plot(rcintime, RCIN.data(:,3)), legend('cmdroll','logroll'), xlabel('sec');

figure; plot(cmdtime, cmd.data(:,3));
hold on; plot(rcintime, RCIN.data(:,4)), legend('cmdpitch', 'logpitch'), xlabel('sec');

%% Plot RC Out Data and vrpn data between two times:
t1 = 40;
t2 = 60;
inds = matchnearest([t1,t2],vrpntime);
figure;
subplot(2,2,1), plot(vrpntime(inds(1):inds(2)),vrpn.data(inds(1):inds(2),2)), xlabel('time(sec)'), ylabel('x(t)');
subplot(2,2,2), plot(vrpntime(inds(1):inds(2)),vrpn.data(inds(1):inds(2),3)), xlabel('time(sec)'), ylabel('y(t)');
subplot(2,1,2), plot(vrpntime(inds(1):inds(2)),vrpn.data(inds(1):inds(2),4)), xlabel('time(sec)'), ylabel('z(t)');
rc_inds = matchnearest([t1,t2],rcouttime);
figure;
subplot(2,2,1), plot(rcouttime(rc_inds(1):rc_inds(2)), RCOU.data(rc_inds(1):rc_inds(2),3)), xlabel('time(sec)'), ylabel('CH_1');
subplot(2,2,2), plot(rcouttime(rc_inds(1):rc_inds(2)), RCOU.data(rc_inds(1):rc_inds(2),4)), xlabel('time(sec)'), ylabel('CH_2');
subplot(2,2,3), plot(rcouttime(rc_inds(1):rc_inds(2)), RCOU.data(rc_inds(1):rc_inds(2),5)), xlabel('time(sec)'), ylabel('CH_3');
subplot(2,2,4), plot(rcouttime(rc_inds(1):rc_inds(2)), RCOU.data(rc_inds(1):rc_inds(2),6)), xlabel('time(sec)'), ylabel('CH_4');
%% Convert RC Out To Throttle, Roll, Pitch and Yaw Torques
%Normalize the pwm to [0,1]
