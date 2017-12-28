run combined_test_0.m

step_average = mean(Step,3);
slope_average = mean(Slope,3);

figure('Name','Data step model');
get_graphs(step_average,3/1000)
figure('Name','Data slope model');
get_graphs(slope_average,20/1000)