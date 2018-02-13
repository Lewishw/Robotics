function [ q_ ] = NullSpaceIK(  robot, xyz_command, ori_des, q )
%OPENLOOPIK Summary of this function goes here
%   Detailed explanation goes here
T_ee = robot.fkine(q);
error = xyz_command - T_ee(1:3,4);
error2 = rotational_error(ori_des,T_ee(1:3,1:3));

J_q = robot.jacob0(q);
J_pos = J_q(1:3,:);
J_wx = J_q(4:6,:);
J_inv = J_pos'* InvertMatrix(J_pos*J_pos'); % Task 1 Pseudo Inverse
J_inv2 = J_wx'* InvertMatrix(J_wx*J_wx'); % Task 2 Pseudo Inverse
q_ = J_inv*error + (eye(6) - J_inv*J_pos)*(J_inv2*error2');
end
