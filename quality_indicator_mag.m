function [ q ] = quality_indicator_mag(x, mag_data)
%QUALITY_INDICATOR Summary of this function goes here
%   Detailed explanation goes here

mag_x_offset = x(1);
mag_x_gain = x(4);
mag_y_offset = x(2);
mag_y_gain = x(5);
mag_z_offset = x(3);
mag_z_gain = x(6);

mag_x_opt = zeros(11449,1); mag_y_opt = mag_x_opt; mag_z_opt = mag_x_opt;

i = 1;
for fi = 0 : pi/53 : 2*pi
    for theta = 0 : pi/106 : pi
        mag_x_opt(i) = 0.49366 * cos(fi) * sin(theta);
        mag_y_opt(i) = 0.49366 * sin(fi) * sin(theta);
        mag_z_opt(i) = 0.49366 * cos(theta);
        i = i + 1;
    end 
end

mag_x = (mag_data(:,1) - mag_x_offset) * mag_x_gain;
mag_y = (mag_data(:,2) - mag_y_offset) * mag_y_gain;
mag_z = (mag_data(:,3) - mag_z_offset) * mag_z_gain;

q = (mean(mag_x_opt) - mean(mag_x)).^2 + (mean(mag_y_opt) - mean(mag_y)).^2 + (mean(mag_z_opt) - mean(mag_z)).^2;

end

