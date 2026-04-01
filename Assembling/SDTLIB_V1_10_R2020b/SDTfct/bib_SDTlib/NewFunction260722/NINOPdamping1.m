% Gout = NINOPdamping1(Gin,nP,DDL,xi)
%
% From the NINOP model Gin, this function overwrites the damping of the
% flexible modes by the common value xi. 
%
% DDL is a vector with the index of d.o.f. considered is the model Gin. 
% Index convention: 1 for Tx, 2 for Ty, 3 for Tz, 
%                   4 for Rx, 5 for Ry, 6 for Rz.
%
% Let us note: Ndof=length(DDL);
%
% INPUT ARGUMENTS:
%
% A generic TITOP model Gin: Ndof(nC+nP) x Ndof(nC+nP) NINOP model between nC child points
% C and nP parent points P.
% It is assumed that the Ndof x Ndof nP channels from the accelerations at points P to 
% the wrenches applied to the parent body at points P is the last  Ndof x Ndof nP channel 
% of Gin.
% It is assumed that the order of Gin is an even interger.
%
% Gin is an SS object (USS are not supported in this function).
%
% OUTPUT ARGUMENTS:
%
% Gout: the same model with all the flexible mode damping ratios equal to
% xi.

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2023)
