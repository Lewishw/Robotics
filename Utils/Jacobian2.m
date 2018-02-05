%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: RobocupUS
%% Created: 2015-04-13
% Jacobian(homo,nb_joint)
%
% Description :
%               This function computes the Jacobian matrix. 
%               It uses equation 3.3 of the page 112 of the book
%               " Robotics, Modelling,Planning and Control "
%
% Inputs :
%        homo           : the homogeneous matrices of each motor related from
%                         the origin
%        nb_joint       : the number of joints(motors) from the DH
%
% Output :
%        J              : The Jacobian matrix
%
% Details:
%        This function works only when all joints are revolute
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ J ] = Jacobian2(DH_robot)

T_0_7 = homogeneous_matrix(DH_robot', 1, length(DH_robot(:,1)));
px = T_0_7(1:3,4);

J(1:3,1) = cross([0;0;1],px);       % position
J(4:6,1)=[0;0;1];                   % angle
for j = 2:length(DH_robot(:,1));
    homo = homogeneous_matrix(DH_robot', 1, j-1);
    z(:,j) = homo(1:3,3);
    p(:,j) = T_0_7(1:3,4)-homo(1:3,4);
    J(1:3,j) = cross(z(:,j),p(:,j));  % position
    J(4:6,j) = z(:,j);                % angle
end