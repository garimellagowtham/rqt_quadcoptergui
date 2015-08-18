
RC_MIN = [1005,1003,1003,1003];
RC_MAX = [1981,1995,1998,1999];
RC_TRIM = [1513, 1514, 1031, 1510];
N = size(rcinput,1);
rc_cmdangle = zeros(N,2);
rctime = (rcinput(:,1) - vrpn.data(1,1))/1e9;
for i = 1:N
   if rcinput(i,2) > RC_TRIM(1)+1
    rc_cmdangle(i,1) = map_data(rcinput(i,2),RC_TRIM(1)+1, RC_MAX(1), 0, pi/4);
   elseif rcinput(i,2) < RC_TRIM(1) - 1
     rc_cmdangle(i,1) = map_data(rcinput(i,2), RC_MIN(1), RC_TRIM(1) - 1, -pi/4, 0);
   else
      rc_cmdangle(i,1) = 0;
   end
   if rcinput(i,3) > RC_TRIM(2)+1
    rc_cmdangle(i,2) = -map_data(rcinput(i,3),RC_TRIM(2)+1, RC_MAX(2), 0, pi/4);
   elseif rcinput(i,3) < RC_TRIM(2) - 1
     rc_cmdangle(i,2) = -map_data(rcinput(i,3), RC_MIN(1), RC_TRIM(2) - 1, -pi/4, 0);
   else
      rc_cmdangle(i,2) = 0;
   end
end
% Plot imu angles:
figure; subplot(2,1,1), hold on, plot(imutime, imu.data(:,2)*(180/pi)), ylabel('roll');
plot(rctime, rc_cmdangle(:,1)*(180/pi), 'r');
subplot(2,1,2), hold on, plot(imutime, imu.data(:,3)*(180/pi)), ylabel('pitch');
plot(rctime, rc_cmdangle(:,2)*(180/pi),'r');