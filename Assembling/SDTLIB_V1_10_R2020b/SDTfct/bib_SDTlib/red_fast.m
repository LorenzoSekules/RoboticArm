% [SYS_OUT]=RED_FAST(SYS_IN,W)
%  truncate, in the modal realization of SYS_IN (LTI system or USS), the
%  eigenvalues whose real parts are lower than the prescribed value W
%  (negative value, rad/s). The direct feedthrough of the reduced system 
%  SYS_OUT is updated in order to SYS_OUT and SYS_IN have the same
%  DC gain.

% D. Alazard 01/95
% Copyright (c) 1993-2000 ONERA/DCSD, All Rights Reserved.
% Revised 03/2011, 03/2020
