% M=TwoPortBeamFull(ro,s,l,e,g,iz,iy,xi) computes the 2 input/output
% model M (12x12) of a uniform beam characterized by:
%   * mass density: ro  (Kg/m^3),
%   * section: s (m^2),
%   * lenght: l (m),
%   * Young modulus: e (Pascal or N/m^2),
%   * shear modulus: g (Pascal or N/m^2),
%   * second moment of area w.r.t z axis: iz (m^4),
%   * second moment of area w.r.t y axis: iy (m^4),
%   * xi: arbitrary damping ratio for all flexible modes.
%          ^ y(x)
%          |
%          |
%          x=======================x---------> x
%         /P(x=0)                 C(x=l)
%        /
%       z
%
%   The 12 inputs of M are:
%      * the 6 components of the external force/torque vector applied
%        to the beam at point C (in the frame (P,x,y,z)),
%      * the 6 components of the linear/angular acceleration vector at
%        point P (in the frame (P,x,y,z)).
%   The 12 outputs of M are:
%      * the 6 components of the linear/angular acceleration vector at
%        point C (in the frame (P,x,y,z)),
%      * the 6 components of the external force/torque vector applied
%        by the beam at point P (in the frame (P,x,y,z)).
%   Only pure flexion in the plane (P,x,y), in the plane (P,x,z) traction and
%   torsion along (P,x) axis are considered.
%   This fonction supports uncertain parameters (see ureal).
%
%   See also: MKelemu

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2015)
