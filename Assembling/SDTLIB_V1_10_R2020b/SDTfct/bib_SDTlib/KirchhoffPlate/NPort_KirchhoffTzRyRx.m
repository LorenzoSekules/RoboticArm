% [MtzRyRx,omega,Lp] = NPort_KirchhoffTzRyRx(l1,l2,t,rho,E,ni,nx,ny,P_id,C_ids,xi)
%
% Finite Element Model MtzRyRx (6x6) of a uniform Kirchhoff plate characterized by:
%   * plate length along x: l1 (m),
%   * plate length along y: l2 (m),
%   * plate thickness along z: t (m),
%   * mass density: rho  (Kg/m^3),
%   * Young modulus: E (Pascal or N/m^2),
%   * Poisson ratio: ni,
%   * number of 4-node plate elements along x: nx,
%   * number of 4-node plate elements along y: ny,
%   * ID number of parent node P: P_id (see NOTE for nodal convention),
%   * ID number of children nodes (vector): C_ids (see NOTE for nodal convention),
%   * xi: arbitrary damping ratio for all flexible modes.
%
%          ^ z    y
%          |     /  
%          |    /             C (x=0, y=ly) 
%          |   -----------------------/
%          |  /                  *   / 
%          | * P(xp = 0,yp)         / 
%          |/                      / 
%          .----------------------/----------> x
%     (x=0, y=0)                
%       
%       
%   Only pure flexion in the plane (P,x,z) and torsion in the plane (P,y,z) are considered.
%
%   The 6 inputs of MtzRyRx are:
%      * the external force F_Cz (along z) and torques T_Cx (around x) 
%        and T_Cy (around y) applied to the plate at point C,
%      * the linear ddot(z)_P (along z) and angular ddot(theta_x)_P
%        (around x) and ddot(theta_y)_P (around y) accelerations at point P.
%   The 6 outputs of M are:
%      * the linear ddot(z)_C (along z) and angular ddot(theta_x)_C
%        (around x) and ddot(theta_y)_C (around y) accelerations at point C,
%      * the external force F_Pz (along z) and torques T_Px (around x) 
%        and T_Py (around y) applied by the plate at point P.
%
%  NOTE: Example for a 4x5 elements plate: global node convention
%
%  25----26----27----28----29----30
%   | 16 |  17 |  18 |  19 |  20 |
%  19----20----21----22----23----24
%   | 11 |  12 |  13 |  14 |  15 |
%  13----14----15----16----17----18
%   | 6  |  7  |  8  |  9  |  10 |
%   7----8-----9-----10----11----12
%   | 1  |  2  |  3  |  4  |  5  |
%   1----2-----3-----4-----5-----6
%
%  Outputs:
%  MtzRyRx : NINOP Kirchhoff Model
%  omega   : modes' frequency (diag matrix) 
%  Lp      : Modal Participation Factors
% 
% More details on the model can be found in: 
% Sanfedino, F., Alazard, D., Pommier-Budinger, V., Falcoz, A., & 
% Boquet, F. (2018). Finite element based N-Port model for preliminary 
% design of multibody systems. Journal of Sound and Vibration, 415, 128-146.
%

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2017)
