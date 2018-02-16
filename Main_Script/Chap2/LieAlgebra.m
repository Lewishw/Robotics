%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% Lie Algebra
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

% Consider a x-axis rotation expressed as a rotation mat
R = roty(0.5)
pause()
% We can compute the logartihm of this matrix using logm
S = logm(R)
pause()
% The result is a sparse matrix with 2 elements that have a magnitude equal
% to the original rotation angle. This matrix has a zero diagonal and is
% another example of a skew-symmetrix matrix(in this case 3x3)

% Applying vex to the skew-symmetric mat gives
vex(S)'
pause()
% And we find the original rotation angle that is the first element
% (corresponds to the x axis).

% The toolbox function trlog is equivalent but gives the angle and rotation
% axis separately
[th,w] = trlog(R)
pause()


%%
%  The inverse of a log is exponentiation and using built-in function expm:
expm(S)
pause()
% We have regenerated our original rotation matrix

% Therefore, rotx(0.3) is equivalent to expm( skew([1 0 0])*0.3);
A = rotx(0.3); B = trexp( skew([1 0 0])*0.3);

disp(A) 
fprintf('==') 
disp(B)

% This generalizes to rotation about any axis