function [ q_ ] = JacobianTransposeIK( robot, xyz_command, q , K2)
%OPENLOOPIK Summary of this function goes here
%   Detailed explanation goes here
    T_ee = robot.fkine(q);
    ori_des = T_ee(1:3,1:3);
    eps = 0.0005;
    error = xyz_command - T_ee(1:3,4);
    error2 = rotational_error(ori_des,T_ee(1:3,1:3));

    q_ = 0*q';
    K = 0.1; % Convergence rate factor
    while norm(error) > eps
        J_q = robot.jacob0(q+q_');
        J_pos = J_q(1:3,:);
        J_w = J_q(4:6,:);
        J_inv = J_pos'* InvertMatrix(J_pos*J_pos'); % Task 1 Pseudo Inverse
        q_ = q_ + J_pos'*(K*eye(3) * error) + (eye(6) - J_inv*J_pos)*(J_w'*(K2*eye(3) * error2'));
        T_ee = robot.fkine(q+q_');
        error = xyz_command - T_ee(1:3,4);
        error2 = rotational_error(ori_des,T_ee(1:3,1:3));
    end
end

