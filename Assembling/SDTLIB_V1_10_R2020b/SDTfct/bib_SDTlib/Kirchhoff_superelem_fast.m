% MFzTxTy = Kirchhoff_superelem_fast(lx, ly, t, rho, E, ni, xi, P_xy, C_xy)
% 
% This function compute the NINOP model of a plate clamped at 
% point P and free at points C
%
% Inputs:
%   * lx (m): length along x-axis,
%   * ly (m): width along y-axis,
%   * t  (m): thickness along z-axis,
%   * rho (Kg/m^3): mass density,
%   * E (N/m^2): Young moddulus,
%   * ni: Poisson's coeficient,
%   * xi: flexible mode common damping ratio,
%   * P_xy (m): coordinates of point P in the (x,y)-plane (1x2),
%   * C_xy (m): coordinates of the n points C in the (x,y)-plane (nx2),
%
% Outputs:
%   * MDzTxTy: the 6(n+1)x6(n+1) NINOP model where the last 6x6 channel 
%     corresponds to the point P.

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2018)
