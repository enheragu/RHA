% First run .m of the test to process
% https://es.mathworks.com/videos/extracting-dynamic-models-from-experimental-data-using-system-identification-spanish-100499.html

step_average = mean(Step,3);
slope_average = mean(Slope,3);

%speed vs time in step test with data combined from n test
figure('Name','Measured Data in step test');
y_axys_step_combined = step_average(:,1);
x_axys_step_combined = step_average(:,3);
plot(x_axys_step_combined,y_axys_step_combined,'r')
hold on
%speed vs time in step test with data from just one test
y_axys_step_single = Step(:,1,1);
x_axys_step_single = Step(:,3,1);
plot(x_axys_step_single,y_axys_step_single,'b')

legend('Media de 20 test','Datos de un test')

%speed vs time in slope test with data combined from n test
figure('Name','Measured Data in slope test');
y_axys_slope_combined = slope_average(:,1);
x_axys_slope_combined = slope_average(:,3);
plot(x_axys_slope_combined,y_axys_slope_combined,'r')
hold on
%speed vs time in slope test with data from just one test
y_axys_slope_single = Slope(:,1,1);
x_axys_slope_single = Slope(:,3,1);
plot(x_axys_slope_single,y_axys_slope_single,'b')
legend('Media de 20 test','Datos de un test')

figure('Name','Slope test');
y_axys_slope_speed = slope_average(:,1);
x_axys_slope_torque = slope_average(:,2);
plot(x_axys_slope_torque,y_axys_slope_speed,'r')
hold on
legend('Speed vs torque applied')