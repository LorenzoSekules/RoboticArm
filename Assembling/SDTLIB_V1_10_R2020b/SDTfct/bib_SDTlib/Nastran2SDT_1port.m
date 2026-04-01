%  [M, J, PG, LS, omega, DPa0, flagFatal] = Nastran2SDT_1port(F06filename,nf)
%
% This function imports from the Nastran .f06 file the data required in 
% the SDTlib block "One-port Nastran Body (6x6)".
%
% INPUT ARGUMENTS
% F06filename           Name of NASTRAN input file with extension .f06
% nf                    Number of flexible modes to be considered (must be
%                       lower than the "NUMBER OF ROOTS FOUND" in the f06
%                       file).
%
% OUTPUT ARGUMENTS
% M                     Mass [1] (Kg)
% J                     Inertia matrix at CG [3x3] (Kgm^2)
% PG                    vector from anchorage point P to Center of gravity G [1x3] (m)
% LS                    Matrix of modal participation factors in translation 
%                       and rotation [nFlex x 6]  (sqrt(Kg) and sqrt(Kg).m)
% omega                 Vector of vibration frequencies [nFlex x 1] (rd/s)
% DPa0                  Residual mass matrix [6x6] (Kgm^2)
% flagFatal             Flag to determine whether the NASTRAN run was successful or not [1]

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2017)
