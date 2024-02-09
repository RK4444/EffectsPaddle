%% Phaser test
fs=44100;
Ts=1/fs;
N=1000;
w1=4000;
w2=8000;
w3=16000;
t=0:Ts:(N-1)*Ts;
tout=0:Ts:(3*N-1)*Ts;
y1=sin(w1.*t);
y2=sin(w2.*t);
y3=sin(w3.*t);
y=[y1 y2 y3];
n1=[0 0 1];
n2=[0 0 0 0 0 0 0 0 1];
d=1;
filt1=filt(n1,d,Ts);
filt2=filt(n2,d,Ts);

yout1=lsim(filt1, y1, t);
yout2=lsim(filt1, y3, t);
grid on;
figure;
subplot(121);plot(t,yout1);
subplot(122);plot(t, yout2);
%% bode
[h1,w1]=freqz(n1, [1 0 0]);
[h2,w2]=freqz(n2, [1 0 0 0 0 0 0 0 0]);
angle1=angle(h1);
angle2=angle(h2);
intpangle=interp1(w1,h1,w2);
plot(w2, intpangle, w2, angle2);

%% flanger test design
nsmall=32;
fs=44100;
Ts=1/fs;
phflang = linspace(-nsmall,nsmall,2*nsmall+1);
phout = (-1)*atan(phflang);

figure("Name","Target Phase");
plot(phflang, phout);
endphase = phout;%fftshift(phout);
endampl = ones(size(endphase));
endprod = endampl.*exp(1j*endphase);
endprod = [endprod(nsmall+1:end) endprod(1:nsmall)]; % because fftshift doesn't shift the right part
imresp = ifft(endprod);

smoothedresp = imresp.*hamming(length(imresp))'; % smoothing filter
figure("Name","Impulse Response");
plot(phflang, smoothedresp);

figure("Name","Smoothed Final Filter");
finalfilttest = filt(10*smoothedresp, 1, Ts); % to bring in Ts
[num, den] = tfdata(finalfilttest);
targetsos = tf2sos(cell2mat(num), cell2mat(den));
freqz(targetsos)

figure("Name","Group Delay");
[hx, wx] = freqz(targetsos);
targetAngle = angle(hx);
gdel=(-1)*gradient(targetAngle, wx); % calculate group delay
plot(wx,gdel);