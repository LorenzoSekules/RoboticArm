% Stable and High-Frequency mode reduction
%
% [SYS_OUT]=RED_SHF(SYS_IN,W)
%  truncate, in the modal realization of SYS_IN (LTI or USS system), the
%  stable eigenvalues whose magnitude are greater than the prescribed value W
%  (positive value, rad/s). 
%
%
% WARNING:!! this function plots also the frequency-domain responses
% (sigma plots) of SYS_IN and SYS_OUT to check if the trucature is
% relevant. 
% 
% [SYS_OUT]=RED_SHF(SYS_IN,W,1) skips this plot.

% D. Alazard 10/2021
% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Revised 03/2020
