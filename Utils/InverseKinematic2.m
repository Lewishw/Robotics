%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: LH
%% Created: 2017-03-15
% InverseKinematic(DH,Pos_d,Ori_d)
%
% Description :
%               This function returns the angular position of each motor to obtain the
%               end-effector desired position and angle.  The
%               equation used to make this function is:
%               equation 4.7 of the book " Advanced robotics "
%
% Inputs :
%        DH             : the Denavit-Hartenberg parameters
%        Pos_d          : Position desired
%        Ori_d          : Desired orientation (RPY)
%
% Output :
%        joint          : the angular position of each motor to obtain the
%                         end-effector desired position and angle
%
% Details :
%           This inverse kinematic function consider 2 priorities:
%           the first priority is the position of the end-effector
%           the second priority is the orientation of the end-effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [joint] = InverseKinematic2(DH,Pos_d,Ori_d)

K = 0.004;         % pseudo-inverse of the Jacobian gain
tol = 0.000001;    % error tolerance
size_ = length(DH);
T_0_7 = homogeneous_matrix(DH, 1, size_);
act_ori = compute_eulerAngles(T_0_7(1:3,1:3),eye(3));
Des_ori = compute_eulerAngles(Ori_d,eye(3));
k1 = 1;
k2 = 0.2;
r1_p = Pos_d - T_0_7(1:3,4);
r2_p = Des_ori - act_ori;
iteration = 0;

% while error is bigger than error tolerance
while(((sum(abs(r1_p)) > tol)||(sum(abs(r2_p)) > tol)) && iteration < 100)
    iteration = iteration+1;
    J = Jacobian2(DH');
    J1 = J(1:3,:);
    J2 = J(4:6,:);
    
    J1_sharp = J1'/(J1*J1'+(K^2*eye(3)));%pseudo-inverse of the jacobian
    % J1_sharp = 1*pinv(J1);%pseudo-inverse of the jacobian
    
    
    A = k1*J1_sharp * r1_p; % first priority
    J2_hat = J2*(eye(length(DH(1,:)))-(J1_sharp*J1));
    J2_hat_sharp = J2_hat'/(J2_hat*J2_hat'+(K^2*eye(3)));
    B = J2 * A;
    C = (r2_p) - B';
    D = k2 * J2_hat_sharp * C';  %second priority
    delta_q = A+D;
    
    DH(4,:) = DH(4,:) + delta_q';
    
    T_0_7 = homogeneous_matrix(DH, 1, size_);
    act_ori = compute_eulerAngles(T_0_7(1:3,1:3),eye(3));
    Des_ori = compute_eulerAngles(Ori_d,eye(3));
    [r1_p,r2_p] = compute_error(T_0_7,Pos_d,Ori_d);% Error recalculation
end
joint = DH(4,:);
