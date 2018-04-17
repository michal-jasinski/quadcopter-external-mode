% close all;

% time = 0 : 0.01 : 0.01 * length(pitch) - 0.01;
% figure(1);
% hold on; grid on;
% plot(time, roll, time, pitch, time, yaw);
% title('Roll-pitch-yaw angles');
% legend('roll', 'pitch', 'yaw');
% xlabel('Time [s]');
% ylabel('Angle [deg]');

 
% load('workspaces\imu_calibration_data3.mat')
% 
% offset_mag = [3.8; -0.38; 0.35];
% gain_mag = [0.9; 1.32; 1.32];
offset_mag = [0; 0; 0];
gain_mag = [1; 1; 1];
% 
% ideal magnetometer elipsoid 
x_mag = zeros(11449,1); y_mag = x_mag; z_mag = x_mag;
i = 1;
for fi = 0 : pi/53 : 2*pi
    for theta = 0 : pi/106 : pi
        x_mag(i) = 0.49366 * cos(fi) * sin(theta);
        y_mag(i) = 0.49366 * sin(fi) * sin(theta);
        z_mag(i) = 0.49366 * cos(theta);
        i = i + 1;
    end 
end

% data is in gauss, where 1 Gauss is 100uT
figure(1);
plot3((mag_x(:,1) - offset_mag(1)) * gain_mag(1),(mag_y(:,1) - offset_mag(2)) * gain_mag(2),(mag_z(:,1) - offset_mag(3)) * gain_mag(3));
% plot3(mag_x, mag_y, mag_z);
hold on;
plot3(x_mag,y_mag,z_mag);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
grid on;
hold off; 

% %% accelerometer_data
% gain_acc = [1; 1; 1];
% offset_acc = [0; 0; 0];
% 
% x_acc = zeros(11449,1); y_acc = x_acc; z_acc = x_acc;
% i = 1;
% for fi = 0 : pi/53 : 2*pi
%     for theta = 0 : pi/106 : pi
%         x_acc(i) = cos(fi) * sin(theta);
%         y_acc(i) = sin(fi) * sin(theta);
%         z_acc(i) = cos(theta);
%         i = i + 1;
%     end 
% end

% data is in gauss, where 1 Gauss is 100uT
% figure(2);
% plot3((acc_x(:,1) - offset_acc(1)) * gain_acc(1),(acc_y(:,1) - offset_acc(2)) * gain_acc(2),(acc_z(:,1) - offset_acc(3)) * gain_acc(3));
% % plot3(mag_x, mag_y, mag_z);
% hold on;
% plot3(x_acc,y_acc,z_acc);
% xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
% grid on;
% hold off; 