step_average = mean(Step,3);
slope_average = mean(Slope,3);

%speed vs time in step test with data combined from n test
figure('Name','Measured Data');
subplot(1,3,1)
hold on
plot(step_average(:,3)/1000/1000,step_average(:,1),'r')
%speed vs time in step test with data from just one test
plot(Step(:,3,1)/1000/1000,Step(:,1,1),'b')
xlabel('Time (seconds)')
ylabel('Speed (rpm)')
title('Plot of data step response')
legend('Average (20 test)','Data from single test')

%speed vs time in slope test with data combined from n test
subplot(1,3,2)
hold on
plot(slope_average(:,3)/1000/1000,slope_average(:,1),'r')
%speed vs time in slope test with data from just one test
plot(Slope(:,3,1)/1000/1000,Slope(:,1,1),'b')
xlabel('Time (seconds)')
ylabel('Speed (rpm)')
title('Plot of data slope response')
legend('Average (20 test)','Data from single test')

%speed vs torque in slope test
subplot(1,3,3)
hold on
plot(slope_average(:,2),slope_average(:,1),'r')
plot(Slope(:,2,1),Slope(:,1,1),'b')
xlabel('Torque (0-1023)')
ylabel('Speed (rpm)')
title('Plot of data slope response (speed vs torque)')
legend('Speed vs torque applied')