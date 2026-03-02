 % getDataFromNASTRANforROFS.m
%-----------------------------------------------------------------
% PURPOSE
% Read data from Patran *.bdf and Nastran output *.f06 files to get the
% data required by the MultiBody/Reduced Order Flexible Solid block.
%
% SYNOPSIS
%  [coord,Mrofs,Krofs,Drofs,flagFatal] = 
%  getDataFromNASTRANforROFS(F06filename,BDFfilename,damping,P_gp,C_gp,nf)
%
% INPUT ARGUMENTS:
%
% F06filename           Name of NASTRAN input file with extension .f06,
% BDFfilename           Name of PATRAN bdf file with extension .bdf,
% damping               Damping ratio - same value for all modes,
% P_gp                  Grid point number corresponding to nodal point P  
%                       (point of attachment of appendage),
% C_gp                  Grid point number corresponding to N nodal points C, 
% nf                    number of flexible modes.
%
% OUTPUT ARGUMENTS:
%
% Coord                 Coordonates ((N+1)x3) of the origine of the N+1 
%                       interface frames attached to points P, C1, ..., CN,
% Mrofs                  Mass matrix to used for the Simscape block: Reduced
%                       Order Flexible Solid,
% Krofs                 Stiffness matrix to used for the Simscape block:
%                       Reduced Order Flexible Solid,
% Drofs                 Damping matrix to used for the Simscape block: 
%                       Reduced Order Flexible Solid,
% flagFatal             Flag to determine whether the NASTRAN run was 
%                       successful or not [1].

% Daniel Alazard - ISAE - 28/10/2021 .


function [coord,Mrofs,Krofs,Drofs,flagFatal]=...
    getDataFromNASTRANforROFS(F06filename,BDFfilename,xi,P_gp,C_gp,nf)

if nf<6*length(C_gp),
    disp('Error: the number of flexible modes must be greater than 6 x N');
    disp('(N = number of nodes C.)');
    coord=[];Mrofs=[];Krofs=[];Drofs=[];flagFatal=1;
    return
end,

rfindex='f';

[M,J,xcg,LS,omega,EV_C,zeta,DPa0,cdP,cdC,tauCP,flagFatal]=...
    getDataFromNASTRAN_Nport(F06filename,BDFfilename,rfindex,xi,P_gp,C_gp,nf);
  
nf=6;
M=m_hexa_truss;
J=inertias_hexa_truss;
xcg=r_PG_hexa_truss;
LS=hexagon_truss_SIMSCAPE_reduced_FEM.L';
omega=wn_hexa_truss;
[EV_C,A]=eig(hexagon_truss_SIMSCAPE_reduced_FEM.M(7:end,7:end),hexagon_truss_SIMSCAPE_reduced_FEM.K(7:end,7:end));
EV_C=EV_C(1:6,:);
zeta=ones(1,nf)'*xi_hexa_truss;
DPa0=hexagon_truss_SIMSCAPE_reduced_FEM.M(1:6,1:6)-hexagon_truss_SIMSCAPE_reduced_FEM.L'*hexagon_truss_SIMSCAPE_reduced_FEM.L;
cdP= hexagon_truss_3p_SIMSCAPE_reduced_FEM.P(1,:);
cdC= hexagon_truss_3p_SIMSCAPE_reduced_FEM.P(3,:);
%xCP=1;
%yCP=1;
%zCP=1;
%tauCP= [eye(3) [0 -zCP yCP; zCP 0 -xCP; -yCP xCP 0]; zeros(3) eye(3)];
xi=0.01;


% Origines of the interface frames:
coord=[cdP;cdC];
% Number of points C:
N=length(C_gp);
% Kinematic model between centre of mass and point P::
tauCGP=kinematic_model(xcg',cdP'); 
% Mass matrix of the hybrid-cantilever (at P) model:
MassMat=[tauCGP'*[M*eye(3) zeros(3);zeros(3) J]*tauCGP LS';LS eye(nf)]; 
% Stiffness matrix of the hybrid-cantilever (at P) model:
StiffMat=[zeros(6,6+nf);zeros(nf,6) diag(omega.^2)];
% Damping matrix of the hybrid-cantilever (at P) model:
DampMat=[zeros(6,6+nf);zeros(nf,6) 2*xi*diag(omega)];

% Kinematic model between the point P and the points C
tauCPi=[];
for i=1:N;tauCPi=[tauCPi;tauCP(:,:,i)];end;

% Sub-space assocociated to X_Ci, i=1,...nC:
phiCi=[];
for i=1:N;phiCi=[phiCi;EV_C(:,:,i)];end;

% check independence of xP, xC1, xC2, ...
if rank(phiCi)<6*N,
    disp('Error: ''xP'' and ''xC''  are not independent.');
    coord=[];Mrofs=[];Krofs=[];Drofs=[];flagFatal=1;
    return
end
if cond(phiCi)>1e9,
    disp('Warning: bad condition number:');
    disp('''xP'' and ''xC''  are not totally independent.');
    disp('results  may be unaccurate');
end

% Input matrix of the hybrid-canteliver (at P) model:
% InMat=[-eye(6) tauCPi';zeros(nf,6) phiC'];
  
% Complementary sub-space:
phiCorth=null(phiCi)';
% Mapping from initial flexible generalized coordinates to new coordinates
% with deltaX_Ci for the 6xnC first components
Pass=[phiCi;phiCorth];
Passm1=inv(Pass);
% Mapping from the inertial (6+nf) d.o.f vector to the new d.o.f with
% [X_P' X_C1' X_C2' ...]' for the 6(nC+1) first components:
Mapmat=[eye(6) zeros(6,nf);...
    Passm1(:,1:6*N)*[-tauCPi eye(6*N)] Passm1(:,6*N+1:nf)];
Mrofs=Mapmat'*MassMat*Mapmat;
Krofs=Mapmat'*StiffMat*Mapmat;
Drofs=Mapmat'*DampMat*Mapmat;
% Check:
% Mapmat'*InMat  % must be equal to :
% [-eye(6) zeros(6,6*N);zeros(6*N,6) eye(6*N);zeros(nf-6*N,12)] !!!
   
end



