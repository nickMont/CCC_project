clc;
close all;
clear all;

n_points = 10:10:100;
time = [0.30 1.04 3.05 4.22 6.93 10.36 14.94 19.78 26.76 34.26];
time2 = [0.56 2.04 4.88 8.96 16.78 20.68 25.87 30.15 37.44 41.08];
time3 = [0.844 3.83 9.17 17.12 27.71 32.99 41.05 46.44 57.85 60.30];
plot(n_points,time,n_points,time2,n_points,time3); grid on; 
xlabel('Number of Points'); 
ylabel('Solution time(s)'); 
title('Solution time for optimal segment Min-snap');
legend('Backtracking Gradient Descent','nlopt','nlopt with ineq. constraints','Location','Best');
time./time2