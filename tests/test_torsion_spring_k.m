% Author: Sarit Nagel
% Date: 27/07/2026

% % This script computes the equivalent torsional spring constant (K_total) for a 
% planar torsion spring consisting of N identical, radially-oriented tapered 
% flexures. It accounts for the varying cross-section of the beams using 
% Euler-Bernoulli beam theory and integration.

clc

%% ------------------ INPUT PARAMETERS ------------------
% Geometry of the torsion spring

R_outer = 33.5e-3;          % Outer radius of flexure (m)
R_inner = 11.68/2*1e-3;     % Inner radius (hub/teeth) (m)

Plate_Thick = 4.75e-3;      % Thickness of plate (out-of-plane dimension)

% Width of one flexure.
% The beam tapers linearly from the teeth to the rim.
Width_inner = 0.75e-3;      % Width at inner hub (m)
Width_outer = 3.15e-3;      % Width at outer rim (m)

N_spokes = 24;              % Number of identical flexures

% Material property
E = 71.7e9;                 % Young's Modulus of Aluminum (Pa)

%% ------------------ FLEXURE GEOMETRY ------------------

% Effective bending length of one flexure
L = R_outer - R_inner;

% Beam width as a function of position along the flexure.
% x = 0 corresponds to the inner hub (teeth)
% x = L corresponds to the outer rim.
% Since the beam is tapered, the width changes continuously.
w = @(x) Width_inner + (Width_outer-Width_inner).*(x/L);

% Second moment of area as a function of position.
%
% For a rectangular cross section:
%
%           I = t*w^3 / 12
%
% Because the width varies along the beam, the moment of inertia
% also varies continuously instead of remaining constant.
I = @(x) Plate_Thick .* w(x).^3 / 12;

%% ------------------ CALCULATE LINEAR STIFFNESS ------------------

% Compute the beam compliance using Euler-Bernoulli beam theory:
%
%       C = ∫ (L-x)^2 / (E*I(x)) dx
%
% This integral sums the flexibility of many small beam segments,
% each with its own local stiffness.
C = integral(@(x) ((L-x).^2)./(E.*I(x)),0,L);

% Stiffness is the inverse of compliance.
k_linear_spoke = 1/C;

%% ------------------ CONVERT TO TORSIONAL STIFFNESS ------------------

% The linear force acting on a spoke creates torque about the center.
%
% Torque = Force × Radius
%
% Therefore the equivalent torsional stiffness is obtained by
% multiplying the linear stiffness by the square of the effective radius.
R_avg = (R_outer + R_inner)/2;

K_spoke = k_linear_spoke * R_avg^2;

% Since all spokes act in parallel, their stiffnesses add together.
K_total = N_spokes * K_spoke;

%% ------------------ DISPLAY RESULTS ------------------

fprintf('Total Torsional k        : %.4f Nm/rad\n',K_total);