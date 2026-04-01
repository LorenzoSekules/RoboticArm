% MODEL = D_sat_at_O(FILENAME) compute the model MODEL of the
%   spacecraft described in the file FILENAME.m 
%
% MODEL is a structure:
%   MODEL.TotalMass: total mass of the spacecraft,
%   MODEL.TotalCg: 3x1 coordinate vector of the global center of mass
%                  (CGtot) in the reference frame attached to the main body
%                  (O, X, Y, Z),
%   MODEL.InverseTotalModel: [(6+NBPIVOTS)x(6+NBPIVOTS) ss] inverse dynamic
%                   model between inputs:
%                    * 6 dof external force-torque applied on the main body,
%                    * NBPIVOTS torques applied inside the NBPIVOTS
%                      pivot joints between bodies,
%                   and outputs:
%                    * 6 dof linear-angular accelerations of main body,
%                    * NBPIVOTS relative angular accelerations of
%                    appendages around pivots axis.
%                    This model is written at main body refernce point 0
%                    in frame (0, X, Y, Z).
%   MODEL.DynamicModel: [(6+NBPIVOTS)x(6+NBPIVOTS) ss] direct dynamic model of 
%                   the spacecraft (the inverse of the previous one).
%   MODEL.liste_IOs: Description and ordering of the (6+NBPIVOTS) inputs
%                   (outputs) used in the dynamic model.
%
%   Reference: 
%       Alazard, D., Cumer, C., and Tantawi, K., 
%       “Linear dynamic modeling of spacecraft with various flexible
%        appendages and on-board angular momentums�?. 
%        In Proceedings of the 7th International ESA Conference on Guidance, 
%        Navigation & Control Systems
%        Tralee, Ireland, 1-5 June 2008

% Copyright (c) ISAE-SUPAERO, All Rights Reserved.
% Daniel Alazard (2019)

%==========================================================================
