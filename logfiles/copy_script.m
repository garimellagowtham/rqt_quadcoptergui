cd session08_May_2014_05_52_24_PM/
RC_MIN = [1108,1103,1105,1101];
RC_MAX = [1932,1929,1926,1926];
cmd = importdata('cmd.dat','\t',1);
%servo = importdata('servo.dat','\t',1);
vrpn = importdata('vrpn.dat','\t',1);
vrpntime = (vrpn.data(:,1) - vrpn.data(1,1))/1e9;
%servotime = (servo.data(:,1) - vrpn.data(1,1))/1e9;
vrpnrpy = quat2rpy([vrpn.data(:,8) vrpn.data(:,5:7)]);
figure(1), clf;
subplot(2,2,1), plot(vrpntime,vrpn.data(:,2)), xlabel('time(sec)'), ylabel('x(t)');
subplot(2,2,2), plot(vrpntime,vrpn.data(:,3)), xlabel('time(sec)'), ylabel('y(t)');
subplot(2,1,2), plot(vrpntime,vrpn.data(:,4)), xlabel('time(sec)'), ylabel('z(t)');
figure(2), clf;
subplot(2,2,1), plot(vrpntime,vrpnrpy(:,1)*(180/pi)), xlabel('time(sec)'), ylabel('roll(t)^o');
subplot(2,2,2), plot(vrpntime,vrpnrpy(:,2)*(180/pi)), xlabel('time(sec)'), ylabel('pitch(t)^o');
subplot(2,1,2), plot(vrpntime,vrpnrpy(:,3)*(180/pi)), xlabel('time(sec)'), ylabel('yaw(t)^o');
if ~exist('imu.dat','file')
    return;
end
imu = importdata('imu.dat','\t',1);
imutime = (imu.data(:,1) - vrpn.data(1,1))/1e9;
figure(2);
subplot(2,2,1),hold on, plot(imutime,imu.data(:,2)*(180/pi),'r');
subplot(2,2,2),hold on, plot(imutime,imu.data(:,3)*(180/pi),'r');
subplot(2,1,2),hold on, plot(imutime,imu.data(:,4)*(180/pi),'r');
if sum(size(cmd.data)) > 0
    cmdtime = (cmd.data(:,1) - vrpn.data(1,1))/1e9;
    rcmd = mapouttoin([RC_MIN(1),RC_MAX(1),-pi/4,pi/4], cmd.data(:,2));
    pcmd = mapouttoin([RC_MIN(2),RC_MAX(2),-pi/4,pi/4], cmd.data(:,3));
    throtcmd = mapouttoin([RC_MIN(3),RC_MAX(3),0,7.357], cmd.data(:,4));
    ycmd = mapouttoin([RC_MIN(4),RC_MAX(4),-1,1], cmd.data(:,5));
    figure(3), clf;
    subplot(2,2,1), hold on, plot(cmdtime, rcmd*(180/pi)), xlabel('time(sec)'), ylabel('rollcmd');
    subplot(2,2,2), hold on, plot(cmdtime, pcmd*(180/pi)), xlabel('time(sec)'), ylabel('pitchcmd');
    subplot(2,2,3), hold on, plot(cmdtime, ycmd), xlabel('time(sec)'), ylabel('rateyawcmd');
    %subplot(2,2,1),hold on, plot(imutime,imu.data(:,2)*(180/pi),'r');
    %subplot(2,2,2),hold on, plot(imutime,imu.data(:,3)*(180/pi),'r');
    subplot(2,2,1), plot(vrpntime,vrpnrpy(:,1)*(180/pi),'r');
    subplot(2,2,2), plot(vrpntime,vrpnrpy(:,2)*(180/pi),'r');
    subplot(2,2,4), hold on, plot(cmdtime, throtcmd), xlabel('time(sec)'), ylabel('throttlecmd');
end
cd ..

%% Frequency domain analysis:
%tstart = 40; tend = 86.1;%Can check periodicity later %20.3
tstart = 26.63; tend =63.4;
%tstart = 100; tend = 126;%Can check periodicity later %20.3
indcmd = (cmdtime > tstart) & (cmdtime < tend);%  session06_May_2014_11_58_45_PM/
indvrpn = (vrpntime > tstart) & (vrpntime < tend);
figure(4), clf, plot(cmdtime(indcmd),rcmd(indcmd),'b'), hold on, plot(vrpntime(indvrpn), vrpn.data(indvrpn,3) - vrpn.data(1,3),'r'),...
        plot(vrpntime(indvrpn), vrpnrpy(indvrpn,1),'g'), legend('rollcmd','vrpny','vrpnroll');
%FFT plots:
samplercmd = rcmd(indcmd);
sampletimecmd = cmdtime(indcmd);
N = sum(indcmd);%Number of samples
T = sampletimecmd(end) - sampletimecmd(1);
Rcmd = fft(samplercmd);
Rcmd = [Rcmd((N/2+1):end) Rcmd(1:(N/2))];
freq = (-N/2:(N/2-1))*(1/T);
figure(5),subplot(2,2,1),  plot(freq,abs(Rcmd)), legend('Rcmd');

samplevrpny = vrpn.data(indvrpn,3);
%Remove dc component
samplevrpny = samplevrpny - mean(samplevrpny);
vrpntimesample = vrpntime(indvrpn);
N = sum(indvrpn);%Number of samples
T = vrpntimesample(end) - vrpntimesample(1);
VrpnY = fft(samplevrpny)';
VrpnY = [VrpnY((N/2+1):end) VrpnY(1:(N/2))]; 
freq = (-N/2:(N/2-1))*(1/T);
figure(5), subplot(2,2,2), plot(freq,abs(VrpnY)), legend('VrpnY');
samplevrpnroll =  vrpnrpy(indvrpn,1);
VrpnRoll = fft(samplevrpnroll)';
VrpnRoll = [VrpnRoll((N/2+1):end) VrpnRoll(1:(N/2))];
freq = (-N/2:(N/2-1))*(1/T);
figure(5), subplot(2,1,2), plot(freq, abs(VrpnRoll)), legend('VrpnRoll');
%% Plotting velocities:
dt = conv(vrpntime,[1;-1],'valid');
disp('Mean Freq: ');
disp(1/mean(dt));
dvx = conv(vrpn.data(:,2),[1;-1],'valid')./dt;
dvy = conv(vrpn.data(:,3),[1;-1],'valid')./dt;
dvz = conv(vrpn.data(:,4),[1;-1],'valid')./dt;
velmag = sqrt(dvx.^2 + dvy.^2 + dvz.^2);
figure(6), clf;
subplot(2,2,1), plot(vrpntime(1:end-1),dvx), xlabel('time(sec)'), ylabel('vx');
subplot(2,2,2), plot(vrpntime(1:end-1),dvy), xlabel('time(sec)'), ylabel('vy');
subplot(2,2,3), plot(vrpntime(1:end-1),dvz), xlabel('time(sec)'), ylabel('vz');
subplot(2,2,4), plot(vrpntime(1:end-1),velmag), xlabel('time(sec)'), ylabel('velmag');