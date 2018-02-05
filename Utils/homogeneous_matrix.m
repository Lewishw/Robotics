%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: RobocupUS
%% Created: 2015-04-13
% homogeneous_matrix(DH,nb_joint)
%
% Description :
%               This function returns the homogeneous matrices of each motor
%               from the base. The construction of the transformation matrices
%               uses equation 2.52 of page 64 of the book
%               " Robotics, Modelling,Planning and Control "
%
% Inputs :
%        DH             : the Denavit-Hartenberg parameters to transfer to
%                         homogeneous matrices
%        nb_joint       : the number of joints(motors) from the DH
%
% Output :
%        homo           : the homogeneous matrices of each motor related from
%                         the origin
%               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ homo ] = homogeneous_matrix(DH,from_joint, to_joint) %

a = DH(1,:);    % a
al = DH(2,:);   % alpha
d = DH(3,:);    % d
th = DH(4,:);   % theta
nb_joint = length(a);
% Construction of transformation matrices
trans = zeros(4:4:nb_joint);
for i = 1:nb_joint
trans(1,:,i) = [cos(th(i)) (-sin(th(i))*cos(al(i))) (sin(th(i))*sin(al(i))) (a(i)*cos(th(i)))];
trans(2,:,i) = [sin(th(i)) (cos(th(i))*cos(al(i))) (-cos(th(i))*sin(al(i))) (a(i)*sin(th(i)))];
trans(3,:,i) = [0 sin(al(i)) cos(al(i)) d(i)];
trans(4,:,i) = [0 0 0 1];
end 

% Homogeneous matrices all related from Origin
homo = zeros(4,4);
homo(:,:) = trans(:,:,from_joint);
for j = (from_joint+1):to_joint
    homo(:,:) = homo(:,:)*trans(:,:,j);  
end