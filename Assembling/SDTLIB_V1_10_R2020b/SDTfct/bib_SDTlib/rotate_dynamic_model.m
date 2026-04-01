% DM_OUT = ROTATE_DYNAMIC_MODEL (DM_IN , ANGLE, AXIS)
% computes the dynamic model after a rotation of ANGLE (deg) 
% around the axis AXIS :
%   * DN_IN:  dynamic model of a body in a frame R,
%   * ANGLE:  rotation angle (deg),
%   * AXIS:   3 components in the frame R of the unitary vector 
%     along the rotation axis,
%   * DN_OUT: dynamic model of the rotated body in the frame R.
%
% It is assumed that the first 6x6 block of DM represent the dynamic model
% between the 6 dof acceleration vector and the 6 dof external force
% vector.

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2015)

