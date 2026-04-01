% function[Kas,Mas] = makeAssembling(nx,ny,Kel,Mel)
%
% Creation of Connectivity Matrix and Assembling of Stiffness and Mass
% matrices for 4nodes-12dof elements
%
% Example for a 4x5 elements plate: global node convention
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
%

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Francesco Sanfedino (2017)
