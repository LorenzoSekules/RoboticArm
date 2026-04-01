% [M, J, xcg, LS, omega, EV_C, zeta, DPa0, cdP, cdC,tauCP] = TITOP2SDT_Nport(G);
%
% This function computes the mechanical and geometrical parameters of system
% from its TITOP model G.
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
% OUTPUT ARGUMENTS:
%
% M                     Mass [1] (Kg)
% J                     Inertia matrix at CG [3x3] (Kg/m^2)
% xcg                   Center of gravity of the body [1x3] (m)
% LP                    Matrix of modal participation factors in translation 
%                       and rotation [nFlex x 6]  (sqrt(Kg) and sqrt(Kg).m)
% omega                 Vector of vibration frequencies [nFlex x 1] (rad/s)
% Phi_C                 Eigen vector/Modal shape at n points C [6n x nFlex]
% zeta                  Vector of damping ratios [nFlex x 1]
% DPa0                  Residual mass matrix [6x6] ([Kg Kg.m;Kg.m Kg.m^2])
% cdP                   Nodal co-ordinates of point P with respect to GFF (m)
% cdC                   Vector of Nodal co-ordiantes of points C with respect to GFF (m) 
% tauCP                 Kinematic model between points C and P [6n x 6]

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2021)
