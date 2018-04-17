function [ q ] = quality_indicator_acc(x, acc_data)
%QUALITY_INDICATOR Summary of this function goes here
%   Detailed explanation goes here

acc_x_offset = x(1);
acc_x_gain = x(4);
acc_y_offset = x(2);
acc_y_gain = x(5);
acc_z_offset = x(3);
acc_z_gain = x(6);

acc_x_opt = zeros(11449,1); acc_y_opt = acc_x_opt; acc_z_opt = acc_x_opt;

i = 1;
for fi = 0 : pi/53 : 2*pi
    for theta = 0 : pi/106 : pi
        acc_x_opt(i) = cos(fi) * sin(theta);
        acc_y_opt(i) = sin(fi) * sin(theta);
        acc_z_opt(i) = cos(theta);
        i = i + 1;
    end 
end

acc_x = (acc_data(:,1) - acc_x_offset) * acc_x_gain;
acc_y = (acc_data(:,2) - acc_y_offset) * acc_y_gain;
acc_z = (acc_data(:,3) - acc_z_offset) * acc_z_gain;

q = (mean(acc_x_opt) - mean(acc_x)).^2 + (mean(acc_y_opt) - mean(acc_y)).^2 + (mean(acc_z_opt) - mean(acc_z)).^2;

end

