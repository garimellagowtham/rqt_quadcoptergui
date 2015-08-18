% Assumes the ctrl variable holds the ctrl.dat data from the controller
%The format of the data is :
%curr_Time	dx[0]	dx[1]	dx[2]	ddx[0]	ddx[1]	ddx[2]	Fext[2]	rpycmd[0]	rpycmd[1]	rpycmd[2]
ctrltime = (ctrl(:,1) - ctrl(1,1))/1e9;
timediff = conv2(ctrltime,[1;-1],'valid');
figure(1), clf, plot(ctrltime(1:end-1),1./timediff);%Plot the frequency 
fprintf(1,'Mean freq: %f\n',mean(1./timediff));

%Plot dx, dy, dz:
figure(2), clf;
subplot(2,2,1), plot(ctrltime,ctrl(:,2)), legend('X'), xlabel('time(sec)');
subplot(2,2,2), plot(ctrltime,ctrl(:,3)), legend('Y'), xlabel('time(sec)');
subplot(2,1,2), plot(ctrltime,ctrl(:,4)), legend('Z'), xlabel('time(sec)');
%Plot ddx, ddy, ddz:
figure(3), clf;
subplot(2,2,1), plot(ctrltime,ctrl(:,5)), legend('vX'), xlabel('time(sec)');
subplot(2,2,2), plot(ctrltime,ctrl(:,6)), legend('vY'), xlabel('time(sec)');
subplot(2,1,2), plot(ctrltime,ctrl(:,7)), legend('vZ'), xlabel('time(sec)');
%%
ind1 = find(abs(ctrltime - 53.47) < 0.01);
ind2 = find(abs(ctrltime - 59.18) < 0.01);

%Sampling freq is 60Hz
%FFT:
N = ind2 - ind1 + 1; %Number of samples
if rem(N,2) ~= 0
    N = N-1;
    ind2 = ind2-1;
end
T = ctrltime(ind2) - ctrltime(ind1); %Seconds
freq = (-N/2:(N/2-1))*(1/T);
y = ctrl(ind1:ind2,2) - mean(ctrl(ind1:ind2,2));%Remove mean to remove dc gain
Y = fft(y);
%Rearrange fft to make more sense
Y = [Y((N/2+1):end); Y(1:(N/2))];
figure(3), clf, plot(freq,abs(Y)), title('Fourier resp'), xlabel('freq(Hz)'), ylabel('Y(f)');
figure(4), clf, plot(freq,phase(Y));
%Windowed signal

%%  Filtering the data:
[b,a] = butter(1,3/(30),'low');
y = ctrl(:,4); 
yfilt = filter(b,a,y);
figure(5),hold on, plot(ctrltime,y);
plot(ctrltime,yfilt,'r');
dy = conv2(y,[1;-1],'valid');
dt = conv2(ctrltime,[1;-1],'valid');
dyfilt = conv2(yfilt,[1;-1],'valid');
figure(6), hold on, plot(ctrltime(1:end-1),dy./dt);
plot(ctrltime(1:end-1),dyfilt./dt,'r');




