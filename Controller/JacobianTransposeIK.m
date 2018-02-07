function [ q_ ] = JacobianTransposeIK( robot, xyz_command, q )
%OPENLOOPIK Summary of this function goes here
%   Detailed explanation goes here
    T_ee = robot.fkine(q);
    eps = 0.005;
    error = xyz_command - T_ee(1:3,4);
    q_ = 0*q';
    K = 0.1;
    while norm(error) > eps
        J_q = robot.jacob0(q+q_');
        J_pos = J_q(1:3,:);
        q_ = q_ + J_pos'*(K*eye(3) * error);
        T_ee = robot.fkine(q+q_');
        error = xyz_command - T_ee(1:3,4);
    end
end

