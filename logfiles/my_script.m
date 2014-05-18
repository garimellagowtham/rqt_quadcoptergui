%cd session12_May_2014_07_38_56_AM/ % YPosn
%cd session12_May_2014_08_08_31_AM/ %X posn
%cd session14_May_2014_03_00_46_AM/ %X Posn extended version
%cd session12_May_2014_08_27_32_AM/ %Z posn
%cd session12_May_2014_02_09_19_AM/ %Single sine with freq = (1/2*pi)Hz
%cd session12_May_2014_06_29_38_AM/
 cd session12_May_2014_01_13_38_AM/
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
% imu = importdata('imu.dat','\t',1);
% imutime = (imu.data(:,1) - vrpn.data(1,1))/1e9;
% figure(2);
% subplot(2,2,1),hold on, plot(imutime,imu.data(:,2)*(180/pi),'r');
% subplot(2,2,2),hold on, plot(imutime,imu.data(:,3)*(180/pi),'r');
% subplot(2,1,2),hold on, plot(imutime,imu.data(:,4)*(180/pi),'r');
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
ctrlr = importdata('ctrl.dat','\t',1);
ctrlrtime = (ctrlr.data(:,1) - vrpn.data(1,1))/1e9;
cd ..
%% Frequency domain analysis:
%tstart = 52.18; tend = 90.16;%Can check periodicity later %20.3 %2_09
%tstart = 18.91; tend = 63.58;%Could not get full window size will have
%some problems 6_29
%tstart = 29; tend = 79;%6_59
%tstart=47.07; tend = 97.08; % 7_38 Y posn
%tstart = 60.22; tend = 110.22;%8_08 X posn
%tstart = 40; tend = 140;
%tstart = 52.19; tend = 90.21; %Z posn
%tstart = 28.14; tend = 128.14; %X posn extended
%tstart = 133; tend = 183; %X posn extended
tstart = 20; tend = 70;
indcmd = find((cmdtime > tstart) & (cmdtime < tend));
indvrpn = find((vrpntime > tstart) & (vrpntime < tend));
axis = 1;%axis 1 - xaxis 2 - yaxis 3 - zaxis
switch axis
    case 1
        figure(4), clf, plot(cmdtime(indcmd),pcmd(indcmd),'b'), hold on, plot(vrpntime(indvrpn), vrpn.data(indvrpn,2) - vrpn.data(1,2),'r'),...
            plot(vrpntime(indvrpn), vrpnrpy(indvrpn,2),'g'), legend('pitchcmd','vrpnx','vrpnpitch');
        samplecmd = rcmd(indcmd);
        samplevrpndata = vrpn.data(indvrpn,2) - vrpn.data(1,2);
        samplevrpnrpydata =  vrpnrpy(indvrpn,2);
    case 2
        figure(4), clf, plot(cmdtime(indcmd),rcmd(indcmd),'b'), hold on, plot(vrpntime(indvrpn), vrpn.data(indvrpn,3) - vrpn.data(1,3),'r'),...
            plot(vrpntime(indvrpn), vrpnrpy(indvrpn,1),'g'), legend('rollcmd','vrpny','vrpnroll');
        samplecmd = rcmd(indcmd);
        samplevrpndata = vrpn.data(indvrpn,2) - vrpn.data(1,2);%This is because the yaw is 90 so that the global x axis is
        % local y axis perturbations
        samplevrpnrpydata =  vrpnrpy(indvrpn,1);
    case 3
        %Substracting mean to scale things properly:
        samplecmd = throtcmd(indcmd) -mean(throtcmd(indcmd));
        samplevrpndata = vrpn.data(indvrpn,4) - mean(vrpn.data(indvrpn,4));
        figure(4), clf, plot(cmdtime(indcmd),samplecmd,'b'), hold on,
        plot(vrpntime(indvrpn), samplevrpndata,'r'),...
            legend('throtcmd','vrpnz');
end
sampletimecmd = cmdtime(indcmd);
N = length(indcmd);%Number of samples
Nhalf = floor(N/2);%When N is odd
T1 = sampletimecmd(end) - sampletimecmd(1);
Freqcmd = fft(samplecmd);
Freqcmd = Freqcmd(1:Nhalf);%Removing the other half as it is symmetric
freq1 = (0:(Nhalf-1))*(1/T1);


vrpntimesample = vrpntime(indvrpn);
N = length(indvrpn);%Number of samples
T2 = vrpntimesample(end) - vrpntimesample(1);
Vrpnfreqdata = fft(samplevrpndata)';
Vrpnfreqdata = Vrpnfreqdata(1:Nhalf);
freq2 = (0:(Nhalf-1))*(1/T2);
if axis ~= 3 %For throttle there is no direct relation with rpy
    figure(5), subplot(2,1,1), plot(freq1,abs(Vrpnfreqdata)), legend('VrpnposFreqdata'), xlim([0,2.1]);
    %figure(5),subplot(2,1,1),  plot(freq2,abs(Freqcmd)), legend('Freqcmd'), xlim([0,2.1]);
    Vrpnrpyfreqdata = fft(samplevrpnrpydata)';
    Vrpnrpyfreqdata = Vrpnrpyfreqdata(1:(Nhalf));
    figure(5), subplot(2,1,2), plot(freq1, abs(Vrpnrpyfreqdata)), legend('VrpnFreqrpydata'), xlim([0,2.1]);
else
    figure(5),subplot(2,1,1),  plot(freq1,abs(Freqcmd)), legend('Freqcmd'), xlim([0,2.1]);
    figure(5), subplot(2,1,2), plot(freq2,abs(Vrpnfreqdata)), legend('VrpnposFreqdata'), xlim([0,2.1]);
end
    %% Fitting Bode Plot:
% Manually Picking the peaks at the corresponding frequencies:
%actual_freqs = [0.0400,   0.0600,   0.1000,   0.1400,   0.2200,   0.3400,   0.4600,   0.7401,   1.2201,   2.0202];
if axis ~=3
    %freq_bode2 = [0.02,0.04, 0.08,0.18,0.22,0.3401,0.4602,0.7403,1.22,2.061]; % 7_38 Y posn observed from the graph
    freq_bode2 = [ 0.04 0.06,0.1,0.14,0.22,0.34,0.46,0.74,1.22,2.02];%0.02 outlier % Xposn
    inds_bode2 = round(freq_bode2*T2)+1;
    freq_bode2 = freq2(inds_bode2);%To get the exact value
    amp_bode = Vrpnfreqdata(inds_bode2)./Vrpnrpyfreqdata(inds_bode2);
    figure(6), semilogx(freq_bode2,20*log10(abs(amp_bode)),'bo-');
else
    freq_bode2 = [0.04,0.06,0.1,0.14,0.22,0.34,0.46,0.7401,1.22,2.03];
    freq_bode1 = [0.04, 0.06,0.1,0.13,0.26,0.32,0.48,0.78,1.22,2.02]; 
    inds_bode2 = round(freq_bode2*T2)+1;
    inds_bode1 = round(freq_bode1*T1)+1;
    freq_bode1 = freq1(inds_bode1);%To get corresponding values
    freq_bode2 = freq2(inds_bode2);%
    amp_bode = Vrpnfreqdata(inds_bode2)./Freqcmd(inds_bode1);
    figure(6), semilogx(freq_bode2,20*log10(abs(amp_bode)),'bo-');
end
%% Finding Transfer Function