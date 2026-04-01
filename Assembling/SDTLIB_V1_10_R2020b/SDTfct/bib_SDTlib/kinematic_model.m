% JACOB=KINEMATIC_MODEL(vec_A, vec_B) calculates the kinematic model JACOB 
% of a body between two points A and B
%   Inputs:
%    * vec_A: 3x1 coordinate vector of point A in a given frame,
%    * vec_B: 3x1 coordinate vector of point B (in the same frame).
%   Output:
%    * JACOB: 6x6 kinematic model (projected in the same frame).
%
%                  | eye(3)      (*AB) |
%          JACOB = |                   |
%                  |zeros(3)     eye(3)|
%

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2008)
