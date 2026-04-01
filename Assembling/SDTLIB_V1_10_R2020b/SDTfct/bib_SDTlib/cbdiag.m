% Gd=CBDIAG(G)
%         CBDIAG Bloc-diagonal representation
%         with 2nr order companion block for complex modes:
%
%              |0           1|            |0|
%         A(i)=|             |  with B(i)=| |  in the single
%              |-w**2  -2*z*w|            |1|
%         
%         input case.
%
%         In the multi-input case, the input matrix Bj(i) of input # j 
%         of the block # i with the highest magnitude is normalized
%         to 1 if the mode # i is real and to  [0 1]' if the mode is
%         complex.
%         (thus the system mist be controllable !!).
%
% [Gd,T]=CBDIAG(G) provides the also the transformation matrix T
%         such that z=Tx where x is the state vector of G and
%         z is the state vector of Gd. This syntax is only meaningful when 
%         G is a state-space model.
%
%
% see also: EIG, CANON

%  REVISED  06/20
