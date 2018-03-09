%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% Page 34 of RVC book edition2
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

T = rotx(pi/2)*roty(pi/2)*rotz(pi/2);
tranimate(T,'3d')

tripleangle
pause()

%%
close all
X = transl(3,4,-4);
angles = 0:0.3:15;
tw = Twist('R',[1 1 1],[3 4 2],0.2);

tranimate(@(theta) tw.T(theta)*X, angles, ...
    'length',0.5, 'retain','rgb','notext')
