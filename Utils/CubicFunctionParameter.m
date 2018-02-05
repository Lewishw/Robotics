%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: RoboCupUS
%% Created: 2015-08-31
% CubicFunctionParameter(Pos_a,Pos_d,tf)
% 
% Description :This function defines a cubic function between two points.
% It returns the 4 parameters of a cubic function describing the
% path between point Pe and Pd. The function considers a 0 velocity at
% times 0 and tf
% 
%
% Inputs :
%           Pos_a   : The initial position (x,y,z) (cm)
%           Pos_d   : The final position (x,y,z) (cm)
%           tf      : The time to achieve the final position (s)
%
% Output :
%           Param  : A 3x4 vector containing the function parameters for
%           all axis. The 3 rows are respectively x,y and z parameters.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Param = CubicFunctionParameter(Pos_a,Pos_d,tf)

a0 = Pos_a;
a1 = zeros(1,length(Pos_a));
a3 = (-2*(Pos_d - Pos_a))./(tf.^3);
a2 = (3*(Pos_d - Pos_a))./(tf.^2);
Param = [a0' a1' a2' a3'];
end
