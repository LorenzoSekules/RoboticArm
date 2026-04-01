% Gout = NINOPdampingU(Gin,nP,DDL,xi,percent,'name')
%
% From the NINOP model Gin, this function overwrites the damping of the
% flexible modes by the common value xi and takes into account relative
% uncertainties on flexible mode frequencies (a comment value: percent).
%
% DDL is a vector with the index of d.o.f. considered as flexible is the 
% model Gin. 
% Index convention: 1 for Tx, 2 for Ty, 3 for Tz, 
%                   4 for Rx, 5 for Ry, 6 for Rz.
%
% Let us note: Ndof=length(DDL);
%
% INPUT ARGUMENTS:
%  
%  1)Gin:
%   A generic TITOP model Gin: Ndof(nC+nP) x Ndof(nC+nP) NINOP model between nC child points
%   C and nP parent points P.
%   It is assumed that the Ndof x Ndof nP channels from the accelerations at points P to 
%   the wrenches applied to the parent body at points P is the last  Ndof x Ndof nP channel 
%   of Gin.
%   It is assumed that the order of Gin is an even interger.
%
%   Gin is an SS object (USS are not supported in this function).
%
%  2)nP: integer = number of parent ports (rk: nP can be null).
%
%  3)DDL: the vectors with the indexes of the DDL considered as flexible in
%   the model Gin.
%
%  4)xi: the common daming ratio.
%
%  5)percent: the common value in % of the uncertainties on each
%  flexible mode frequencies.
%
%  6)'name': (string) the name of the subsystem. (It could be the "name"
%  provided by GCB (with spaces) for a subsystem described by a SIMULINK
%  block).
%
% OUTPUT ARGUMENTS:
%
% Gout: the same model with all the flexible mode damping ratios equal to
% xi.
% The uncertain frequency is named: 'w_name_i' where:
%    * 'name' is the 6)th input argument,
%    * 'i' is the flexible mode number (increasing frequency order).
%    
% Rk: this function can remove some very low significant flexible modes.
% 
% Ref:  "Port inversions of parametric Two-Iput Two-Output Port models of
% flexible substructures", D. Alazard, A. Finozzi, F. Sanfedino, Multobody
% System Dynamics, https://doi.org/10.1007/s11044-023-09883-y 

% See also: Notes_manuscrites_complementsMUBO2023.pdf
% Copyright (c) DyCSyT, All Rights Reserved.
% Daniel Alazard (2023)
