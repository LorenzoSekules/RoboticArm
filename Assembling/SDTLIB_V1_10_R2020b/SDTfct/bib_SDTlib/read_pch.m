% This function reads from pch file the coordinates of the reference points and provides the
% Craig-Bamptom (CB) Mass and Stiffness matrices associated to those grid points
%
% Input:
% file_pch : NASTRAN .pch file
%
% Output:
% coord : coordinates of the boundary nodes
% idx   : boundary nodes mesh indexes
% KK    : CB reduced order stiffness matrix
% MM    : CB reduced order mass matrix

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2021)
