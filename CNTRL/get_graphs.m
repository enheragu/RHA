function [  ] = get_graphs( A, data_time )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%figure('Name','Data step model');
y_axys = A(:,1);
x_axys = A(:,3)/1000/1000;
subplot(3,1,1)
plot(x_axys,y_axys,'r')
title('Plot of data response')
hold
%figure('Name','Mathematical model');
input = A(:,2);
test_data = iddata(y_axys,input,data_time,'InputName','Torque 1023 based','OutputName','Speed','OutputUnit','rpm'); 
sys = tfest(test_data, 1, 0);  %gets tf with 2 poles and 0 zeros
subplot(3,1,2)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',1023);
step(sys,opt)
title('Plot of mathematical model from data (1 pole)')
hold
%figure('Name','Mathematical model');
sys2 = tfest(test_data, 2, 0);  %gets tf with 2 poles and 0 zeros
subplot(3,1,3)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',1023);
step(sys2,opt)
title('Plot of mathematical model from data (2 poles)')
end

