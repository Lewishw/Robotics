%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: LH
%% Created: 2017-03-15
% InverseKinematic(DH,Pos_d)
%
% Description :
%               This function returns the angular position of each motor to obtain the
%               end-effector desired position. 
%
% Inputs :
%        DH             : the Denavit-Hartenberg parameters
%        Pos_d          : Desired end-effector Position
%
% Output :
%        joint          : the angular position of each motor to obtain the
%                         end-effector desired position
% 
% Details :
%           This inverse kinematic function consider only 1 priorities:
%           the position (x,y,z) of the end-effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [joint] = InverseKinematic(DH,Pos_d) 

size_ = length(DH);
K = 0.004;         % Gain of the pseudo-inverse of the Jacobian 
tol = 0.000001;    % error tolerance
T_0_7 = homogeneous_matrix(DH, 1, size_);
erreur_p = Pos_d - T_0_7(1:3,4);
iteration = 0;

% while error is bigger than error tolerance
while(sum(abs(erreur_p)) > tol && iteration < 10) 
    iteration = iteration+1;
    J = Jacobian2(DH');
    J1 = J(1:3,:);    
    J1_sharp = J1'/(J1*J1'+(K^2*eye(3)));%pseudo-inverse of the jacobian
    delta_q = J1_sharp * erreur_p;
    DH(4,:) = DH(4,:) + delta_q';
    
    T_0_7 = homogeneous_matrix(DH, 1, size_);
    erreur_p = Pos_d - T_0_7(1:3,4);
end
joint = DH(4,:);
