%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/08
%
% Simple script to visualize the manipulability ellipsoids of the robot
% in a current configuration
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

%% Load robot model
run mdl_fanuc10L.m

Robot = R;
q = Robot.theta;
% Plot the robot with velocity ellipsoid
h = plot_ellipse(0.1*eye(3),zeros(1,3),'edgecolor','r');
h2 = plot_ellipse(0.1*eye(3),zeros(1,3),'edgecolor','b');
h3 = plot_ellipse(0.1*eye(3),zeros(1,3),'edgecolor','k');
Robot.teach(q)
run_ = 1;
while(run_)
    %% Get robot position from teach interface
    delete(h);delete(h2);delete(h3);
    q = Robot.getpos();
    J = Robot.jacob0(q);
    J_pos = J(1:3,:);
    El_pos = J_pos*J_pos';
    J_w = J(4:6,:);
    El_w = J_w*J_w';
    El_force = inv(El_pos);
    % Normalize the matrix
    El_pos = El_pos/(norm(El_pos(1,:)) +  norm(El_pos(2,:)) + norm(El_pos(3,:)));
    El_w = El_w/(norm(El_w(1,:)) +  norm(El_w(2,:)) + norm(El_w(3,:)));
    El_force = El_force/(norm(El_force(1,:)) +  norm(El_force(2,:)) + norm(El_force(3,:)));
    % Get the position of the ellipse (EE)
    T_EE = Robot.fkine(q);
    C = T_EE.t;
    % Plot the robot with velocity ellipsoid
    h = plot_ellipse(El_pos,C,'edgecolor','r');
    h2 = plot_ellipse(El_w,C,'edgecolor','b');
    h3 = plot_ellipse(El_force,C,'edgecolor','k');
    legend([h, h2, h3],'Velocity','Angular vel','Force');
    error = 0;
    while(error < 0.1)
        pause(1);
        q_last = q;
        try
            q = Robot.getpos();
        catch
            run_ = 0;
            break;
        end
        error = norm(q_last - q);
    end
end