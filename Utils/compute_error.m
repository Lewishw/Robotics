%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: LH
%% Created: 2016-04-13
% compute_error(homo,Pos_d,Ori_d,nb_joint)
%
% Description :
%               This function computes the position and orientation error
%
% Inputs :
%        Transform_mat  : the homogeneous matrices of the end-effector
%        Pos_d          : the desired position of the end-effector
%        Ori_d          : the desired orientation of the end-effector
%
% Output :
%        Pos_e           : the position error
%        Ori_e           : the orientation error     
%               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Pos_e,Ori_e] = compute_error(Transform_mat,Pos_d,Ori_d)

Pos_a = Transform_mat(1:3,4);            % actual position
Ori_a = Transform_mat(1:3,1:3);           % actual orientation
Pos_e = Pos_d-Pos_a;                      % position error = desired position - actual position
Ori_e = rotational_error(Ori_d,Ori_a); % orientation error = desired orientation - actual orientation