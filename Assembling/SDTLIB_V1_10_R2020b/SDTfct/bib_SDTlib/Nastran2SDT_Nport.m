%  [M, J, xcg, LS, omega, EV_C, zeta, DPa0, cdP, cdC,tauCP, flagFatal] = 
%  Nastran2SDT_Nport(F06filename,BDFfilename,rfindex,damping,P_gp,C_gp,nf)
%
%  This function imports from the Nastran .f06 and .bdf files the data 
%  required in the SDTlib block "NINOP Nastran Body (6(n+1)x6(n+1))".
%
% INPUT ARGUMENTS:
%
% F06filename           Name of NASTRAN input file with extension .f06
%
% BDFfilename           Name of PATRAN bdf file with extension .bdf
% rfindex               Index to denote whether the function is invoked for
%                       a rigid or flexible body 'r' --> rigid ,'f' --> flexible        
% damping               Damping ratio - same value for all modes
% P_gp                  Grid point number corresponding to nodal point P (point of 
%                       attachment of appendage and upstream)
% C_gp                  Grid point number corresponding to n nodal points C (where the
%                       modal shape is to be extracted)
% nf                    number of flexible modes
%
% OUTPUT ARGUMENTS:
%
% M                     Mass [1] (Kg)
% J                     Inertia matrix at CG [3x3] (Kg/m^2)
% xcg                   Center of gravity of the body [1x3] (m)
% LS                    Matrix of modal participation factors in translation 
%                       and rotation [nFlex x 6]  (sqrt(Kg) and sqrt(Kg).m)
% omega                 Vector of vibration frequencies [nf x 1] (rad/s)
% EV_C                  Eigen vector/Modal shape at n point C [6 x nFlex x n]
% zeta                  Vector of damping ratios [nFlex x 1]
% DPa0                  Residual mass matrix [6x6] ([Kg Kg.m;Kg.m Kg.m^2])
% cdP                   Nodal co-ordinates of point P with respect to GFF (m)
% cdC                   Vector of Nodal co-ordiantes of points C with respect to GFF (m) 
% tauCP                 Kinematic model between points C and P [6 x 6 x n]
% flagFatal             Flag to determine whether the NASTRAN run was successful or not [1]

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2020, updated 08.04.24)
% 