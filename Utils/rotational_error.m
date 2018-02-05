%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: RobocupUS
%% Created: 2015-04-13
% rotational_error(Ori_d,Ori_a) 
%
% Description :
%               This function returns the orientation error between
%               two frame.
%
% Inputs :
%        Ori_d          : the desired orientation of the end-effector
%        Ori_a          : the actual orientation of the end-effector
%
% Output :
%        Ori_e           : the orientation error     
%               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ori_e = rotational_error(Ori_d,Ori_a) 
r = Ori_d*Ori_a';% equation 3.84 of the page 139 of the book Robotics, Modelling,Planning and Control

% equation 2.23 of the page 52 of the book Robotics, Modelling,Planning and Control
Ori_e(1) = atan2(r(3,2),r(3,3));
Ori_e(2) = atan2(-r(3,1),sqrt(r(3,2)^2+r(3,3)^2));
Ori_e(3) = atan2(r(2,1),r(1,1));
