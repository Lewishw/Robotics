%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Louis Hawley
% Date : 2018/02/04
%
% A demo of frame transformation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear variables;

% Reference frame
Init_frame = transl(0,0,0);

% Translate by a vector
translation_vect = [1,0,3]';
Frame1 = Init_frame*transl(translation_vect);

% Rotate frame 1 and offset z
Frame2 = Frame1*trotz(pi/4)*transl(0,0,-1);

% Rotate frame 2 and offsetx
Frame3 = Frame2*trotx(-pi/2)*transl(0,0,1.5);

% Plot the frames
figure
trplot(Init_frame,'frame','0','color','k','axis',[-2 2 -2 2 0 4])
hold on
trplot(Frame1,'frame','1','color','r')
trplot(Frame2,'frame','2','color','b')
trplot(Frame3,'frame','3','color','m')



%% 

% Reference frame
Init_frame = transl2(0,0);

% C is a vector
C = [1;1];

% We want to rotate around point C using Twist
tw = Twist('R',C);

figure
trplot2(Init_frame,'frame','0','color','k','axis',[-2 4 -2 4])
hold on
plot_point(C,'label','C')
trplot2(tw.T(pi),'frame','1','color','b')

% Lets apply a transform on init frame
T2 = Init_frame * transl2(1,2)*trot2(pi/6);
trplot2(T2,'frame','2','color','r')

% This transformation can be formulated as a Twist
tw2 = Twist(T2);
trplot2(tw2.T(1),'frame','3','color','g')

