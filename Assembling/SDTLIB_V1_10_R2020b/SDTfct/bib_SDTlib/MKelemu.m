% [M,K] = MKelemu(ro,s,l,e,i) computes the mass matrix M (6x6) and the
% stiffness matrix K (6x6) of a uniform beam characterized by:
%   * mass density: ro  (Kg/m^3),
%   * section: s (m^2),
%   * lenght: l (m),
%   * Young modulus: e (Pascal or N/m^2),
%   * second moment of aera: i (m^4).
% M and K are relative to the vector Q of generalized coordinates:
%   Q=[y(0) dy/dx|0 d^2y/dx^2|0 y(l) dy/dx|l d^2y/dx^2|l]^T with
%     * q3=d^2y/dx^2|0;
%     * q4=y(l) - y(0) - l*dy/dx|0;
%     * q5=dy/dx|l - d^2y/dx^2|0;
%     * q6=d^2y/dx^2|l;
%   where y(x) is the lateral deflection at the point of abcisse x along 
%   the beam.
%   Only pure flexion in the plane (x,y) is considered.
%          ^ y(x)
%          |
%          |
%          ========================----------> x
%         /0                      l
%        /
%       z
%   This fonction supports uncertain parameters (see ureal).
%
%   See also: TwoPortBeamTyRz, TwoPortBeam

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2015)
