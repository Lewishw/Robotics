%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% A simple joint-space controller with a orientation task in the null space 
% of a position task.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

%% Load robot model
run mdl_fanuc10L.m
Robot = R;

%% Define a Trajectory
% We simply want to rotate around the base
traj_element = 100;
T_init = Robot.fkine(q0);
Pos_init = T_init.t;

final_pos(1:3) = [cos(pi/2)*Pos_init(1) - sin(pi/2)*Pos_init(2), sin(pi/2)*Pos_init(1) + cos(pi/2)*Pos_init(2), Pos_init(3)];
q_end = JacobianTransposeIK(Robot,final_pos',q0,0.4,0.005);

% Joint Space Trajectory
q_traj = jtraj(q0,q0+q_end',traj_element); % Quintic joints trajectory
Robot.plot(q0);

for I=1:traj_element
   % Position command
   Robot.plot(q_traj(I,:));
   pause(0.05)
end

