%  [coord,Mrofs,Krofs,Drofs,flagFatal] = Nastran2ROFS_150523(F06filename,BDFfilename,damping,P_gp,C_gp,nf)
%
% This function imports from the Nastran .f06 and .bdf files the data required in 
% the Simscape Multibody block "Reduced Order Flexible Solid".
% INPUT ARGUMENTS:
%
% F06filename           Name of NASTRAN input file with extension .f06,
% BDFfilename           Name of PATRAN bdf file with extension .bdf,
% damping               Damping ratio - same value for all modes,
% P_gp                  Grid point number corresponding to nodal point P  
%                       (point of attachment of appendage),
% C_gp                  Grid point number corresponding to N nodal points C, 
%                       Rk: C_gp can be empty (C_gp=[]);
% nf                    number of flexible modes.
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
% flagFatal             Flag to determine whether the NASTRAN run was 
%                       successful or not [1].

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2021)
