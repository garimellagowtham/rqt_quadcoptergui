%cd session12_May_2014_07_38_56_AM/ % YPosn
%cd session12_May_2014_08_08_31_AM/ %X posn
%cd session14_May_2014_03_00_46_AM/ %X Posn extended version
%cd session12_May_2014_08_27_32_AM/ %Z posn
%cd session12_May_2014_02_09_19_AM/ %Single sine with freq = (1/2*pi)Hz
%cd session12_May_2014_06_29_38_AM/
% cd session30_July_2014_05_04_48_PM/ %% Attempt to Use Camcallback
%cd session30_July_2014_06_10_38_PM/ Found a bug that when switching to
%camctrl has a delay of 3 seconds (HUGE) That is a code problem
%cd session31_July_2014_03_37_13_PM/
%cd session05_August_2014_08_58_55_PM/
%cd session08_August_2014_11_55_36_PM/
%cd session07_August_2014_10_03_46_PM/ %Trial with arm opening and all
%cd session07_August_2014_09_59_22_PM/
%cd session11_August_2014_11_42_27_PM/ %Automatic Arm closing manual
%movement
%cd session06_September_2014_11_57_36_AM/
%cd session18_August_2014_08_42_01_PM/
%cd session19_August_2014_04_56_17_PM/ %Very good posn control
%cd session20_August_2014_11_58_47_AM/
%cd session20_August_2014_06_00_21_PM/
%cd session21_August_2014_05_48_47_PM/
%cd session22_August_2014_06_05_33_PM/
%cd session22_August_2014_06_05_33_PM/
%cd session22_August_2014_04_40_26_PM/
%cd session25_August_2014_08_06_11_PM/
%cd session26_August_2014_06_12_12_PM/
%cd session27_August_2014_04_27_24_PM/
%cd session02_September_2014_08_11_36_PM/
%cd session06_September_2014_11_57_36_AM/
%cd session16_September_2014_03_10_23_PM/
%cd session19_September_2014_08_35_01_PM/
%cd session19_September_2014_08_35_01_PM/
%cd session20_September_2014_03_19_03_PM/
%cd session20_September_2014_03_04_47_PM/
%cd session21_September_2014_05_14_28_PM/
%cd session21_September_2014_12_52_43_PM/
%cd session21_September_2014_03_11_34_PM/
%cd session23_September_2014_12_19_37_PM/
%cd session23_September_2014_06_48_28_PM/
%cd session24_September_2014_07_21_32_PM/
%cd session24_September_2014_07_28_53_PM/
cd session24_September_2014_07_28_53_PM/
%cd session21_September_2014_03_11_34_PM
%cd session20_September_2014_03_19_03_PM/
%cd session06_September_2014_11_57_36_AM/ (Have more tipposns if needed)
correct_bias = false;
RC_MIN = [1093,1090,1106,1097];
RC_MAX = [1916,1914,1936,1923];
cmd = importdata('cmd.dat','\t',1);
ctrlr = importdata('ctrl.dat','\t',1);
%servo = importdata('servo.dat','\t',1);
vrpn = importdata('vrpn.dat','\t',1);
vrpntime = (vrpn.data(:,1) - vrpn.data(1,1))/1e9;
%servotime = (servo.data(:,1) - vrpn.data(1,1))/1e9;
vrpnrpy = quat2rpy([vrpn.data(:,8) vrpn.data(:,5:7)]);
if correct_bias
    vrpnrpy(:,1) = vrpnrpy(:,1) - vrpn.data(:,9);
    vrpnrpy(:,2) = vrpnrpy(:,2) - vrpn.data(:,10);
end
% for count1 = 1:3
%     vrpnrpy(:,count1) = vrpnrpy(:,count1) - bias_vrpn(count1);
% end
figure(1), clf;
subplot(2,2,1), plot(vrpntime,vrpn.data(:,2)), xlabel('time(sec)'), ylabel('x(t)');
subplot(2,2,2), plot(vrpntime,vrpn.data(:,3)), xlabel('time(sec)'), ylabel('y(t)');
subplot(2,1,2), plot(vrpntime,vrpn.data(:,4)), xlabel('time(sec)'), ylabel('z(t)');
figure(2), clf;
subplot(2,2,1), plot(vrpntime,vrpnrpy(:,1)*(180/pi)), xlabel('time(sec)'), ylabel('roll(t)^o');
subplot(2,2,2), plot(vrpntime,vrpnrpy(:,2)*(180/pi)), xlabel('time(sec)'), ylabel('pitch(t)^o');
subplot(2,1,2), plot(vrpntime,vrpnrpy(:,3)*(180/pi)), xlabel('time(sec)'), ylabel('yaw(t)^o');
if exist('imu.dat','file')
    imu = importdata('imu.dat','\t',1);
    imutime = (imu.data(:,1) - vrpn.data(1,1))/1e9;
    figure(2);
    subplot(2,2,1),hold on, plot(imutime,imu.data(:,2)*(180/pi),'r');
    subplot(2,2,2),hold on, plot(imutime,imu.data(:,3)*(180/pi),'r');
    subplot(2,1,2),hold on, plot(imutime,imu.data(:,4)*(180/pi),'r');
end
if exist('campose.dat','file')
    campose = importdata('campose.dat','\t',1);
    if ~iscell(campose)
        camtime  = (campose.data(:,1) - vrpn.data(1,1))/1e9;
        figure(10), 
        subplot(2,2,1), plot(camtime,campose.data(:,2)), xlabel('time(sec)'), ylabel('x(t)');
        subplot(2,2,2), plot(camtime,campose.data(:,3)), xlabel('time(sec)'), ylabel('y(t)');
        subplot(2,1,2), plot(camtime,campose.data(:,4)), xlabel('time(sec)'), ylabel('z(t)');
    end
end
if ~iscell(ctrlr)
    if sum(size(ctrlr.data)) > 0
        ctrlrtime = (ctrlr.data(:,1) - vrpn.data(1,1))/1e9;
         %rcmd = ctrlr.data(:,9);
         %pcmd = ctrlr.data(:,10);
        rcmd = ctrlr.data(:,11);
        pcmd = ctrlr.data(:,12);
        %cmdtime = (cmd.data(:,1) - vrpn.data(1,1))/1e9;
        %rcmd = mapouttoin([RC_MIN(1),RC_MAX(1),-pi/4,pi/4], cmd.data(:,2));
        %pcmd = mapouttoin([RC_MIN(2),RC_MAX(2),-pi/4,pi/4], cmd.data(:,3));
        %throtcmd = mapouttoin([RC_MIN(3),RC_MAX(3),0,7.357], cmd.data(:,5));
        %ycmd = mapouttoin([RC_MIN(4),RC_MAX(4),-1,1], cmd.data(:,4));
        %if correct_bias
         %   rcmd = rcmd + vrpn.data(:,9);
          %  pcmd = pcmd + vrpn.data(:,10);
        %end
        figure(3), clf;
        subplot(2,1,1),  plot(ctrlrtime, rcmd*(180/pi)), xlabel('time(sec)'), ylabel('rollcmd');
        subplot(2,1,2), plot(ctrlrtime, pcmd*(180/pi)), xlabel('time(sec)'), ylabel('pitchcmd');
        %subplot(2,2,3), hold on, plot(ctrclclrtime, -ycmd), xlabel('time(sec)'), ylabel('rateyawcmd');
        %subplot(2,2,1),hold on, plot(imutime,imu.data(:,2)*(180/pi),'r');
        %subplot(2,2,2),hold on, plot(imutime,imu.data(:,3)*(180/pi),'r');
        subplot(2,1,1), hold on, plot(vrpntime,vrpnrpy(:,1)*(180/pi),'r');
        subplot(2,1,2), hold on, plot(vrpntime,vrpnrpy(:,2)*(180/pi),'r');
        %subplot(2,2,4), hold on, plot(cmdtime, throtcmd), xlabel('time(sec)'), ylabel('throttlecmd');
    end
end
if exist('tippos.dat','file')
    tippos = importdata('tippos.dat','\t',1);
end
if exist('plotfilt.dat','file')
    posfilt = importdata('plotfilt.dat');
end
cd ..


%% Find the object position in optitrack frame:
objposinquadframe = campose.data(:,2:4);
idx = matchnearest(camtime,vrpntime);
vrpnobj_quat = [vrpn.data(idx,8) vrpn.data(idx,5:7)];
vrpnposduringobject = vrpn.data(idx,2:4);
objposinoptframe = quatrotate(vrpnobj_quat,objposinquadframe)+vrpnposduringobject;
figure(11), clf;
subplot(2,2,1), plot(camtime,objposinoptframe(:,1),'DisplayName','objx(t)'), xlabel('time(sec)');
subplot(2,2,1),hold on, plot(camtime,vrpn.data(idx,2),'r','DisplayName','quadx(t)'),legend('show');
subplot(2,2,2), plot(camtime,objposinoptframe(:,2),'DisplayName','objy(t)'), xlabel('time(sec)');
subplot(2,2,2),hold on, plot(camtime,vrpn.data(idx,3),'r','DisplayName','quady(t)'),legend('show');
subplot(2,1,2), plot(camtime,objposinoptframe(:,3),'DisplayName','objz(t)'), xlabel('time(sec)');
subplot(2,1,2),hold on, plot(camtime,vrpn.data(idx,4),'r','DisplayName','quadz(t)'),legend('show');
armwrt_quad = [0.0732, 0, -0.1];
% Also find the tip position using the tipfile:
if exist('tippos','var')
    tiptime  = (tippos.data(:,1) - vrpn.data(1,1))/1e9;
    idx2 = matchnearest(tiptime,vrpntime);
    vrpnobj_quat2 = [vrpn.data(idx2,8) vrpn.data(idx2,5:7)];
    tipposinopt_frame = quatrotate(vrpnobj_quat2,(tippos.data(:,2:4) + ones(size(idx2))*armwrt_quad)) + vrpn.data(idx2,2:4) ;
    figure(11), 
    subplot(2,2,1),hold on, plot(tiptime,tipposinopt_frame(:,1),'g','DisplayName','tipx(t)'),legend('show');
    subplot(2,2,2),hold on, plot(tiptime,tipposinopt_frame(:,2),'g','DisplayName','tipy(t)'),legend('show');
    subplot(2,1,2),hold on, plot(tiptime,tipposinopt_frame(:,3),'g','DisplayName','tipz(t)'),legend('show');
end
%% Plot Current goal in vrpn:
figure(1),
subplot(2,2,1), hold on, plot(vrpntime,vrpn.data(:,11),'r');
subplot(2,2,2), hold on, plot(vrpntime,vrpn.data(:,12),'r');
subplot(2,1,2), hold on,  plot(vrpntime,vrpn.data(:,13),'r');
%% Find target location for arm and see that in optitrack frame
desiredtipposinopt_frame = quatrotate(vrpnobj_quat2,(tippos.data(:,5:7) + ones(size(idx2))*armwrt_quad)) + vrpn.data(idx2,2:4) ;
figure(11),
subplot(1,3,1),hold on, plot(tiptime,desiredtipposinopt_frame(:,1),'m','DisplayName','destipx(t)'),legend('show');
subplot(1,3,2),hold on, plot(tiptime,desiredtipposinopt_frame(:,2),'m','DisplayName','destipy(t)'),legend('show');
subplot(1,3,3),hold on, plot(tiptime,desiredtipposinopt_frame(:,3),'m','DisplayName','destipz(t)'),legend('show');
% idx3 = matchnearest(camtime,tiptime);
% object_offset =ones(size(idx3))*[0, -0.08 -0.18];
% vrpnobj_quat3 = [vrpn.data(idx2,8) -vrpn.data(idx2,5:7)];
% target_location = objposinquadframe + quatrotate(vrpnobj_quat3,object_offset) - ones(size(idx3))*armwrt_quad;
% target_location_inoptframe = quatrotate(vrpnobj_quat2,target_location + ones(size(idx3))*armwrt_quad) + vrpn.data(idx2,2:4);
%% Plotting x,y z velocities:
vrpnvel = conv2(vrpn.data(:,1:4),[1;-1],'valid');
%vrpnvel(:,2:4) = (120)*vrpnvel(:,2:4);
vrpnvel(:,2:4) = 1e9*(vrpnvel(:,2:4)./[vrpnvel(:,1) vrpnvel(:,1) vrpnvel(:,1)]);
vrpnvel(:,1) = [];
figure(13), clf;
subplot(2,2,1), plot(vrpntime(1:end-1), vrpnvel(:,1));
subplot(2,2,2), plot(vrpntime(1:end-1), vrpnvel(:,2));
subplot(2,1,2), plot(vrpntime(1:end-1), vrpnvel(:,3));
%Smoothing using butterworth filter of freq 1Hz:
%% Kalman filter data plot from c++
posfilttime = (posfilt(:,1) - vrpn.data(1,1))/1e9;
figure(13);
subplot(2,2,1), hold on, plot(posfilttime, posfilt(:,5), 'r');
subplot(2,2,2), hold on, plot(posfilttime, posfilt(:,6), 'r');
subplot(2,1,2), hold on, plot(posfilttime, posfilt(:,7), 'r');
figure(1),
subplot(2,2,1), hold on, plot(posfilttime, posfilt(:,2), 'r');
subplot(2,2,2), hold on, plot(posfilttime, posfilt(:,3), 'r');
subplot(2,1,2), hold on, plot(posfilttime, posfilt(:,4), 'r');
%% Putting a threshold on rpy data to see if the difference in two roll,
%values is not more than 20 degrees:
vrpndiff = conv2(vrpnrpy,[1;-1]);
figure;
subplot(2,2,1), plot(vrpndiff(:,1)*(180/pi));
subplot(2,2,2), plot(vrpndiff(:,2)*(180/pi));
subplot(2,1,2), plot(vrpndiff(:,3)*(180/pi));
%% Plot bias data:
figure(14), subplot(2,1,1), plot(vrpntime, vrpn.data(:,9)*180/pi,'DisplayName','bias_roll'), legend('show');%Bias_roll
subplot(2,1,2), plot(vrpntime, vrpn.data(:,10)*180/pi,'DisplayName','bias_pitch'), legend('show');%Bias_roll
%% Plotting throttle cmd to figure out the conversions:
cmdtime = (cmd.data(:,1) - vrpn.data(1,1))/1e9;
throtcmd = cmd.data(:,5);
figure(15), hold on, plot(cmdtime, throtcmd,'c');
%% Throttle Calibration:
% First find the acceleration of quadcopter - external acceleration(
% gravity)
vrpnacc = conv2(vrpn.data(:,2:4),[1;-2;1],'valid');
vrpntimediff = conv2(vrpntime,[1;-1],'valid');
vrpntimediffsqr = vrpntimediff(1:end-1).*vrpntimediff(2:end);
vrpnacc = vrpnacc./(vrpntimediffsqr*ones(1,3));
gravity = [0, 0, -9.81];
mass_quad = 1.76;%Can be 1.66 if small battery
vrpnthrotforce = mass_quad *(vrpnacc - ones(size(vrpnacc,1),1)*gravity);%in Newtons
figure(16), clf;
subplot(2,2,1), plot(vrpntime(1:end-2), vrpnacc(:,1));
subplot(2,2,2), plot(vrpntime(1:end-2), vrpnacc(:,2));
subplot(2,1,2), plot(vrpntime(1:end-2), vrpnacc(:,3));
figure(17), clf;
quiver3(vrpn.data(2:end-1,2),vrpn.data(2:end-1,3), vrpn.data(2:end-1,4),vrpnthrotforce(:,1),vrpnthrotforce(:,2), vrpnthrotforce(:,3));
%% Plot Fext[0,1,2]
figure(18),
subplot(2,2,1), plot(ctrlrtime,ctrlr.data(:,8));
subplot(2,2,2), plot(ctrlrtime,ctrlr.data(:,9));
subplot(2,1,2), plot(ctrlrtime,ctrlr.data(:,10));
%% Find the bias in roll, pitch:
%First find the acceleration in x, y and find zero crossings (There is
%noise so be careful. We use velocity peaks instead. Find the roll, pitch at these zero crossings and
%average them out to find the biases. This works as long as thrust > 0
%which is true in our cases. Also since x,y is in inertial frame, we need
%yaw to be zero or very close so that the relations between roll,y and
%pitch, x are valid.
[b,a] = butter(5,2*2/90.01);%2Hz cutoff freq
vrpnvel_filter(:,1) = filter(b,a,vrpnvel(:,1));
vrpnvel_filter(:,2) = filter(b,a,vrpnvel(:,2));
vrpnvel_filter(:,3) = filter(b,a,vrpnvel(:,3));
figure(13);
subplot(2,2,1), hold on, plot(vrpntime(1:end-1), vrpnvel_filter(:,1),'r');
subplot(2,2,2), hold on, plot(vrpntime(1:end-1), vrpnvel_filter(:,2),'r');
subplot(2,1,2), hold on, plot(vrpntime(1:end-1), vrpnvel_filter(:,3),'r');
timeshift = 0.42;%Find this manually for now:
[peaks_velx,idx] = findpeaks(abs(vrpnvel_filter(:,1)),'MINPEAKHEIGHT',0.01,'MINPEAKDISTANCE',150);
[peaks_vely,idy] = findpeaks(abs(vrpnvel_filter(:,2)),'MINPEAKHEIGHT',0.01,'MINPEAKDISTANCE',150);
idx = idx - timeshift/80;
idy = idy - timeshift/80;
idx(idx<=0) = [];
idy(idy<=0) = [];
%[peaks_velz,idz] = findpeaks(abs(vrpnvel_filter(:,3)),'MINPEAKHEIGHT',0.01,'MINPEAKDISTANCE',150);
%Find roll, pitch values at idx:
disp('roll: ');
disp(mean(vrpnrpy(idy,1))*(180/pi));
disp('pitch: ');
disp(vrpnrpy(idx,2));
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