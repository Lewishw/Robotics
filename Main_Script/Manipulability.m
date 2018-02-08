%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/08
%
% Simple script to visualize the manipulability of the robot in a current 
% configuration
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

%% Load robot model
run mdl_fanuc10l.m
Robot = R;
Robot.teach()

while(1)
%% Set robot in a random position
q = Robot.getpos();
J = Robot.jacob0(q);
J_pos = J(1:3,:);
El_pos = J_pos*J_pos';
J_w = J(4:6,:);
El_w = J_w*J_w';
% Normalize the matrix
El_pos = El_pos/(norm(El_pos(1,:)) +  norm(El_pos(2,:)) + norm(El_pos(3,:)));
El_w = El_w/(norm(El_w(1,:)) +  norm(El_w(2,:)) + norm(El_w(3,:)));

% Get the position of the ellipse (EE)
T_EE = Robot.fkine(q);
C = T_EE(1:3,4);

% Plot the robot with velocity ellipsoid
h = plot_ellipse(El_pos,C,'edgecolor','r');
h2 = plot_ellipse(El_w,C,'edgecolor','b');
disp('Press key to continue')
pause;
delete(h);delete(h2);
end