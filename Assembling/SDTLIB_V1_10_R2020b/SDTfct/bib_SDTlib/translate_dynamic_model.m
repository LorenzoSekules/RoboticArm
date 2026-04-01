% TDM = TRANSLATE_DYNAMIC_MODEL(vec_A,vec_B,DM) translate the direct
% dynamic model from point A to point B in the same reference frame.
%   Inputs:
%    * vec_A: 3x1 coordinate vector of point A,
%    * vec_B: 3x1 coordinate vector of point B,
%    * DM:  dynamic model at point A.
%   Output:
%    * TDM: dynamic model at point B.
%
% It is assumed that the first 6x6 block of DM represent the dynamic model
% from the 6 dof acceleration vector to the 6 dof external force
% vector.

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2015)
