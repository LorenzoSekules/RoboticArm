% Gred = TITOPreduc(G);
%
% This function removed in the TITOP model G, the flexible modes with non
% unitary rank residue.
%
% INPUT ARGUMENT:
%
% A generic TITOP model G: 6(n+1) x 6(n+1) TITOP model between n child points
% C and one parent point P.
% It is assumed that the 6x6 channel from the acceleration at point P to 
% the wrench applied to the parent body at point P is the last 6x6 channel 
% of G.
% It is assumed that the order of G is an even interger.
%
% G is an SS object (USS are not supported in this function).
%
% OUTPUT ARGUMENTS:
%
% Gred: the reduced TITOP model

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2021)
