%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/01/31
%
% Robot Calibration Procedure
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear variables; close all;
% Lets suppose I have a function F(x,y) = ax + by, but I don't know the coefficient a and b
% I set some x and y and look at the results
A = [4.3 7.2];
A_est = [ 3.5    7.56];
num_measurement = 10;
measurement = zeros(1,num_measurement);
command_vect = zeros(num_measurement,2);
error_vect = zeros(1, num_measurement);
A_mat = zeros(num_measurement,2);
for I=1:num_measurement
    meas_noise = (randi(1000,1,1)-500)/1000;
    command_error1 = (randi(100,1,1) - 50)/1000;
    command_error2 = (randi(100,1,1)-50)/1000;
    command_vect(I,:) = [randi(100,1,1)-50 , randi(100,1,1)-50];
    measurement(I) = meas_noise + A*[command_error1 + command_vect(I,1); 
                                               command_error2 + command_vect(I,2)]; 
    error_vect(I) =  measurement(I) - A_est*   command_vect(I,:)'; 
    A_mat(I,:) = A_est;
end

error = 1;
counter = 1;
eps = 0.01
while(error > eps && counter < 5000)  
    A_est = A_est + (inv(command_vect'*command_vect)*command_vect' * error_vect')';
    error_vect = measurement - A_est*command_vect';
    error = sum(error_vect);
    counter = counter + 1;
end

A_est


