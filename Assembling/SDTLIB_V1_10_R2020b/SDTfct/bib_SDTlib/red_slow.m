% [SYS_OUT]=RED_SLOW(SYS_IN,W)
%  truncate, in the modal realization of SYS_IN (LTI or USS system), the
%  eigenvalues whose magnitude are lower than the prescribed value W
%  (positive value, rad/s). 
%
% WARNING:!! this function plots also the frequency-domain responses
% (sigma plots) of SYS_IN and the difference SYS_IN-SYS_OUT to check 
% if the trucature is relevant (maybe long for uss). 
% 
% [SYS_OUT]=RED_SLOW(SYS_IN,W,1) skips this plot.

% D. Alazard 01/2012
% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Revised 03/2020
