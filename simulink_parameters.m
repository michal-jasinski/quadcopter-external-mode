clearvars;
close all;

% I2C defines
% Who am I
WHO_AM_I = hex2dec('F');
LSM6DS33_WHO_AM_I_OK = hex2dec('69');
LIS3MDL_WHO_AM_I_OK = hex2dec('3D');

% Magnetometer data registers
LIS3MDL_OUTX_L = hex2dec('28');
LIS3MDL_OUTX_H = hex2dec('29');
LIS3MDL_OUTY_L = hex2dec('2A');
LIS3MDL_OUTY_H = hex2dec('2B');
LIS3MDL_OUTZ_L = hex2dec('2C');
LIS3MDL_OUTZ_H = hex2dec('2D');

% Accelerometer data registers
LSM6DS33_OUTX_L_XL = hex2dec('28');
LSM6DS33_OUTX_H_XL = hex2dec('29');
LSM6DS33_OUTY_L_XL = hex2dec('2A');
LSM6DS33_OUTY_H_XL = hex2dec('2B');
LSM6DS33_OUTZ_L_XL = hex2dec('2C');
LSM6DS33_OUTZ_H_XL = hex2dec('2D');

% Gyroscope data registers
LSM6DS33_OUTX_L_G = hex2dec('22');
LSM6DS33_OUTX_H_G = hex2dec('23');
LSM6DS33_OUTY_L_G = hex2dec('24');
LSM6DS33_OUTY_H_G = hex2dec('25');
LSM6DS33_OUTZ_L_G = hex2dec('26');
LSM6DS33_OUTZ_H_G = hex2dec('27');

LSM6DS33_ADDRESS = 214;
LIS3MDL_ADDRESS = 60;



% load('workspaces\data.mat');
% 
% t = transpose(0 : 0.001: 10);
% 
q = 0.0001;r = 1;
Q = [0.8 0;0 0.1]*q;   % niepewnoœæ procesu (szumy procesowe i niepewnoœæ modelowania)
R = r;                 % niepewnoœæ pomiaru (im mniejsze tym bardziej ufamy pomiarom)
        
dt = 0.001;
A = [1 -dt;0 1];
B = [dt; 0];
C = [1 0];

Pk_corr_0 = Q; 

mag_x_offset = single(3.48);
mag_y_offset = single(-0.162);
mag_z_offset = single(0.177);

mag_x_gain = single(1.17);
mag_y_gain = single(1.133);
mag_z_gain = single(1.14);

acc_x_offset = single(0.004);
acc_y_offset = single(0.0);
acc_z_offset = single(0.021);

acc_x_gain = single(0.975);
acc_y_gain = single(0.99);
acc_z_gain = single(0.97);

trigger_init = 1;
q_init = single([0 0 0 0]);