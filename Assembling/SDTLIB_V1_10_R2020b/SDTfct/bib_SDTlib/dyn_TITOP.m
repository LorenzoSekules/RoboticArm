
% [w,Lp,dif] = dyn_TITOP(G);
%
% This function provides the:
%  * mode's frequencies, 
%  * the modal participation factors at the connection point P with the 
%     parent body,
%  * and the residual mass 
% of a generic TITOP model of size 6 by 6 (transfer from accelerations to wrench vector). 
%
% Input:
% G : Transfer function from 6 accelerations to wrench vector (3 forces and
% 3 torques) at one given point
%
% Output:
% w   : Modal frequencies (rad/s)
% Lp  : Modal Participation Factors
% dif : Residual mass (must almost null).

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2021)
