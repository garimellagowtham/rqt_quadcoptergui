%Script
[t,u,freqsinp,phases] = generatesumofsineslogspaced(N,f0,10,0.1,2);
t = t';
figure(1), clf, plot(t,u);
u1 = u*(N/2);
amplitude = ceil(max(abs(u1))/0.30);
u1 = u1*(1/amplitude);
max(abs(u1));
freqsinp'
(freqsinp/f0)'
phases'

u1der = conv2(u1,[1;-1],'valid')./conv2(t,[1;-1],'valid');
max(abs(u1der))
figure(2), clf, plot(t(1:end-1),u1der);
U1 = fft(u1);
figure(3),clf, plot(abs(U1));