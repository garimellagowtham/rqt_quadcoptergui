function [ t,u, freqs,phases ] = generatesumofsineslogspaced( N,f0, Numsines,attenuationcoeff,fmax )
%generatesumofsines: Generates sum of sines function using ifft. This is
%more efficient than summing up sines at different frequencies.
% f0 Hz is the input signal frequency which is also the window frequency
%The attenuationcoeff should be very close to 0 but positive to reduce the
%amplitude of high frequency input
Utilde = zeros(N,1);%N sampmaxles per period
loginds = logspace(log10(f0),log10(min(N*f0/2,fmax)),Numsines);%We do not want to go to full N/2 to avoid aliasing effects
loginds = nextprime(round(loginds/f0));
phases = rand(10,1);
Utilde(loginds) = (2./(loginds.^attenuationcoeff))'.*exp(2*pi*(1i)*phases);%Random phase and frequencies are 2:256 fractions 
%inversely proportional to frequency
freqs = (loginds*f0)';
% of the sampling frequency
u = 2*real(ifft(Utilde));
fs = N*f0;%Number of points per period times the signal frequency
t = 0:(1/fs):(1/f0 - 1/fs);%time sequence
%plot(t,u);
end

