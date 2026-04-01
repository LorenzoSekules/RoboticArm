% Mi=invioAug(M,IND) inverses the input/output channel IND 
% in the square system M (same numbers of inputs and ouputs).
%     * M can be a state-space or a transfer model or a uss,
%     * Mi is a state-space model or a uss,
%     * IND can be a vector of indices,
%     * M and Mi have the same numbers of inputs and outputs in the same order.
%
% Mi=invioAug(M,OUT,IN) inverses the channel corresponding to
% M(OUT,IN).
%     * M can be a state-space or a transfer model or a uss,
%     * Mi is a state-space model or a uss,
%     * OUT and IN are a vector of indices with the same lenght,
%     * M and Mi have the same numbers of inputs and outputs in the same order.

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (20015)
