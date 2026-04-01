%  [coord,Mrofs,Krofs,Drofs] = TITOP2ROFS(G)
%
% From the TITOP model G, this function computes the data required in the Simscape Multibody block
% "Reduced Order Flexible Solid" to model a flexible body
%
% INPUT ARGUMENTS:
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
% Coord                 Coordonates ((N+1)x3) of the origine of the N+1 
%                       interface frames attached to points P, C1, ..., CN,
% Mrofs                  Mass matrix to used for the Simscape block: Reduced
%                       Order Flexible Solid,
% Krofs                 Stiffness matrix to used for the Simscape block:
%                       Reduced Order Flexible Solid,
% Drofs                 Damping matrix to used for the Simscape block: 
%                       Reduced Order Flexible Solid,

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2021)
