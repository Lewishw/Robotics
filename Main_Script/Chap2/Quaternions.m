%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% Quaternion (page 44)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;
% The quaternion is an extension of the complex number- a hypercomplex number - \n and is written as a scalar + a vector :
%          q = s + v1*i + v2*j + v3*k 
%              where orthogonal complex numbers i,j,k -> i^2=j^2=k^2=ijk=-1 
% And we denote a quaternion as q = s<v1, v2, v3> ;
% To represent rotation, we used unit quaternion ||q|| = 1. 
% They can be considered as rotation of theta about the unit vector  v 

%In toolbox, we use UnitQuaternion class : \n q = UnitQuaternion(rpy2tr(0.1,0.2,0.3)
q = UnitQuaternion(rpy2tr(0.1,0.2,0.3))
pause()


%Multiplication : q = q*q 
q = q*q
pause()

% Inversion, conjugate of quaternion : inv(q) 
inv(q)
pause()

% Multiplication by its inverse yiels unit quaternion: q = q*inv(q) \n')
q*inv(q)
% Which represents a null rotation or more succintly q/q 
q/q
pause()

% Convert back to orthonormal rotation matrix qith q.R
q.R
pause()


% Rotate a vector by a quaternion
q*[1 0 0]'
pause()
% We can plot the orientation of a quaternion with q.plot()
q.plot()
pause()
