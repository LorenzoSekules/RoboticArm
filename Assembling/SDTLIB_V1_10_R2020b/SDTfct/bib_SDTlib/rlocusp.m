%RLOCUSP Root locus for a square system 
%       (same number of inputs and outputs).
%
%       RLOCUSP(SYS) where SYS is a continuous or discrete-time 
%       LTI system computes and plots the locus of eigenvalues of:
%
%            inv(I+k*SYS)
%
%       for a set of gains k automatically selected.
%       The closed-loop dynamics for the nominal loop gain (k=1) 
%       is marked with blue '+'.
%
%       R=RLOCUSP(SYS,K) allows the vector K of loop gains to be
%       taken into account and returns the matrix R of the closed-loop
%       eigenvalues for the loop gains given in K.
%       (R is LENGTH(K) columns and LENGTH(A) rows)
% 
%       other syntax:
%             * RLOCUSP(A,B,C,D)
%             * RLOCUUSP(A,B,C,D,K)
%
%       See also: RLOCUS, PZMAP

% D. Alazard 01/94
