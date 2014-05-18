%cd session12_May_2014_07_38_56_AM/ % YPosn
%cd session12_May_2014_08_08_31_AM/ %X posn
cd session14_May_2014_06_04_55_AM/
%cd session12_May_2014_08_27_32_AM/ %Z posn
%cd session12_May_2014_02_09_19_AM/ %Single sine with freq = (1/2*pi)Hz
%cd session12_May_2014_06_29_38_AM/
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
% tstart = 50, tend = 200;
count = 0;
Numberofsines = 10;
Umean = zeros(Numberofsines,1);
Ymean = zeros(Numberofsines,1);
Ysqr = zeros(Numberofsines,1);
Usqr = zeros(Numberofsines,1);
YUsqr = zeros(Numberofsines,1);
freq_bode = [ 0.04 0.06,0.1,0.14,0.22,0.34,0.46,0.74,1.22,2.02];%0.02 outlier % Xposn
stop_loop = false;
while ((count < 20)&&~stop_loop)%Almost like while(1) loop
    tstart = 50 + count*25;
    tend = tstart + 50;%Window size 50
    count = count + 1;
    if tend > 199%200
        %count = 30;
        stop_loop = true;
        tend = 200;
        tstart = 150;
    end
    %tstart = 133; tend = 183; %X posn extended
    %tstart = 20; tend = 70;
    indcmd = find((cmdtime > tstart) & (cmdtime < tend));
    indvrpn = find((vrpntime > tstart) & (vrpntime < tend));
    axis = 3;%axis 1 - xaxis 2 - yaxis 3 - zaxis
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
    %freq_bode2 = [0.02,0.04, 0.08,0.18,0.22,0.3401,0.4602,0.7403,1.22,2.061]; % 7_38 Y posn observed from the graph 
    inds_bode2 = round(freq_bode*T2)+1;
     inds_bode1 = round(freq_bode*T1)+1;
   % freq_bode2 = freq2(inds_bode2);%To get the exact value Not needed here
    Y = transpose(Vrpnfreqdata(inds_bode2));
    U = transpose(Freqcmd(inds_bode2));
    Umean = ((count-1)*Umean + U)/count;
    Ymean = ((count-1)*Ymean + Y)/count;
    Usqr = Usqr + U.*conj(U);
    Ysqr = Ysqr + Y.*conj(Y);
    YUsqr = YUsqr + Y.*conj(U);
    pause;
end
figure(6),subplot(2,1,1), plot(freq_bode,abs(Ymean),'b*-');
subplot(2,1,2), plot(freq_bode,abs(Umean),'b*-');
Ghat = Ymean./Umean; %Transfer Function Response
figure(7), semilogx(freq_bode,20*log10(abs(Ymean./Umean)),'bo-');
%Adding Variance Code
assert(count ~= 1);
Uvar = (1/(count-1)).*Usqr - (count/(count-1)).*(Umean.*conj(Umean));
Yvar = (1/(count-1)).*Ysqr - (count/(count-1)).*(Ymean.*conj(Ymean));
YUvar = (1/(count-1)).*YUsqr - (count/(count-1)).*(Ymean.*conj(Umean));
Gvar = ((1/count).*(Ghat.*conj(Ghat))).*(((Yvar)./(Ymean.*conj(Ymean))) + ((Uvar)./(Umean.*conj(Umean))) - 2.*real(YUvar./(Ymean.*conj(Umean))));
figure(8),semilogx(freq_bode,Gvar,'r'),xlabel('freq(Hz)'),ylabel('Variance(G) for the current expt');

%% Finding Transfer Function
options = optimset('MaxFunEvals',1e5,'MaxIter',1e5,'Display','iter');
%Only testing Model 8 and Model 5
modelnumber = 5; paramcount = 6;% thetamin = [-6.7402; -0.4024; 4.1999; 34.6942; 3.0928; -1.4033] MLE val = 8.827240
%modelnumber = 8; paramcount = 4;%thetabest =[-0.0628;-4.6633;-0.0222;-0.1365]; MLE val = 5.6393
%modelnumber = 9; paramcount = 3;%thetabest = [-1.5851;-0.3605;1.3460]
thetamin = zeros(paramcount,1);
fvalmin = 1e6;
s = zpk('s',0);%svariable
for count = 1:50
    theta0 = unifrnd(-10,10,paramcount,1);%2 parameters from -4 to 4
    if theta0(paramcount) < 0
        theta0(paramcount) = -theta0(paramcount);
    end
    [thetastar,fval] = fminsearch(@(x) Transferfunction(x,freq_bode',Ghat,modelnumber),theta0,options);
    if fval < fvalmin
        fvalmin = fval;
        thetamin = thetastar;
    end
end
fprintf(1,'Modelnumber: %f\tFvalue: %f,\n',modelnumber, fvalmin);
disp('thetamin:');
disp(thetamin);
if(thetamin(paramcount)<0)
    thetamin(paramcount) = 0;
end

%Modelnumber = 5
G1 = (exp(-s*thetamin(6))*thetamin(1))*((s^2+thetamin(2)*s+thetamin(3))/(s^2+thetamin(4)*s+thetamin(5))); 
%Modelnumber = 8
%G1 = (exp(-s*thetamin(4))*thetamin(1))*((s+thetamin(2))/(s^2+thetamin(3)));
%Modelnumber  = 9
%G1 = (exp(-s*thetamin(3))*thetamin(1))/(s^2+thetamin(2));
[H,W] = bode(G1,freq_bode);
H = squeeze(H);
W = rem(squeeze(W)*pi/180,2*pi);
W(W>pi) = W(W>pi) - 2*pi;
W(W<-pi) = W(W<-pi)+ 2*pi;
figure(9), clf;
figure(9),subplot(2,1,1), semilogx(freq_bode,20*log10(abs(Ghat)),'b'),xlabel('Freq(Hz)'), ylabel('Ghat(dB)vsGmodel(dB)'), hold on;
figure(9),subplot(2,1,2), semilogx(freq_bode,rem(phase(Ghat),2*pi),'b'),xlabel('Freq(Hz)'), ylabel('phase(rad)vsphasemodel(rad)'), hold on;
figure(9),subplot(2,1,1), semilogx(freq_bode,20*log10(abs(H)),'r'), hold on;
figure(9),subplot(2,1,2), semilogx(freq_bode,W,'r'), hold on;