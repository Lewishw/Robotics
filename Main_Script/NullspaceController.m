%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% A simple null-space controller with a orientation task in the null space 
% of a position task.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

%% Load robot model
run mdl_fanuc10l.m
Robot = R;

%% Define a Trajectory
% We simply want to rotate around the base
traj_element = 100;
T_init = Robot.fkine(q0);
Pos_init = T_init(1:3,4);
rot_inc = pi/traj_element;
q = q0;
pos = Pos_init;
cos_angle = cos(rot_inc);
sin_angle = sin(rot_inc);

% Initialisation
command_plot = zeros(3,traj_element);
result_plot = zeros(3,traj_element);
RotError_plot = zeros(3,traj_element);
plotter = Robot_Plotter(Robot, q);
Robot.plot(q);
ori_des = [0 0 -1; 0 1 0; 1 0 0];
for I=1:traj_element
   % Position command
   command(1:3) = [cos_angle*pos(1) - sin_angle*pos(2), sin_angle*pos(1) + cos_angle*pos(2), Pos_init(3) - 0.5*(sin(rot_inc*I))];
   q = q + (NullSpaceIK(Robot, command', ori_des, q))';
   Robot.plot(q);
   pause(0.05)
   pos = command;
   command_plot(:, I) = pos;
   result = Robot.fkine(q);
   result_plot(:,I) = result(1:3,4);
   error2 = rotational_error(ori_des,result(1:3,1:3));
   RotError_plot(:,I) = error2;
end

figure
plot(command_plot(1,:), command_plot(2,:), 'ro')
hold on
plot(result_plot(1,:), result_plot(2,:), 'bo')
legend('Command','Actual')
axis equal
title('End effector position')



%% Plot the tracking error
figure 
norm_vect = zeros(1,traj_element);
for I = 1: traj_element
    norm_vect(I) = norm(command_plot(1:2,I) - result_plot(1:2,I));
end
plot(norm_vect)
title('Pos Error')
ylabel('Norm Pos error (m)')


% Orientation error
figure 
for I = 1: traj_element
    norm_vect(I) = norm(RotError_plot(:,I));
end
plot(norm_vect)
title('Orientation Error')
ylabel('Norm Orientation error (rad)')
