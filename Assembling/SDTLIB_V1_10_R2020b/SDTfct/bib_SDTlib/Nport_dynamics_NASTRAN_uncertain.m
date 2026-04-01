
% Nport_dynamics_NASTRAN_uncertain build a NINOP model from 
% nb_mode : number of modes
% LS      : Matrix of modal participation factors in translation 
%           and rotation [nFlex x 6]
% omega   : Vector of vibration frequencies [nFlex x 1]
% EV_C    : Eigen vector/Modal shape at n point C [6 x nFlex x n]
% zeta    : Vector of damping ratios [nFlex x 1]
% DPa0    : Residual mass matrix [6 x 6]
% tauCP   : Kinematic model between points C and P [6 x 6 x n]
% n_free  : Number of C points

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2020)
 