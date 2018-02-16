function [ q_ ] = OpenLoopIK_pos( robot, xyz_command, q )
%OPENLOOPIK Summary of this function goes here
%   Detailed explanation goes here
    T_ee = robot.fkine(q);
    error = xyz_command - T_ee.t;
    J_q = robot.jacob0(q);
    J_pos = J_q(1:3,:);
    J_inv = J_pos'* InvertMatrix(J_pos*J_pos'); % Pseudo Inverse
    q_ = J_inv * error;
end

