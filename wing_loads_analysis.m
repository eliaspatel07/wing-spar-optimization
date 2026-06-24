%% =======================================================================
% Wing Spar Structural Analysis and Weight Optimization MATLAB Model
% ========================================================================
%
% Purpose: This model performs a structural analysis of a semi-span
% aircraft wing spar that has been subjected to maneuver loading. My
% objective is to estimate internal loads, stresses, deflections, and
% factors of saftey based on theory learned in MECHENG 211: Introduction 
% to Solid Mechanics, prior to CAD modeling and Finite Element Analysis.
%
% ========================================================================
%
% Project Assumptions: Before performing the analysis, several simplifying 
% assumptions are made to keep the problem manageable while still 
% capturing the primary structural behavior of the wing spar.
%
% 1. The aircraft wing is modeled as a cantilever beam fixed at
% the fuselage and free at the wing tip.
%
% 2. Only one half of the wing is analyzed since the aircraft is
% assumed to be symmetric.
%
% 3. Lift is modeled using an ideal elliptical distribution,
% which is commonly used as a first approximation for wing
% loading.
%
% 4. The aircraft is evaluated using a certification-style maneuver case.
% A 2.5g limit load factor is used and multiplied by 1.5 to obtain the
% corresponding 3.75g ultimate load condition.
%
% 5. Wing structural weight is assumed to be uniformly
% distributed along the span.
%
% 6. Fuel weight is assumed to be uniformly distributed within
% the inboard fuel tank region.
%
% 7. The spar is represented as a tapered I-beam with dimensions
% that vary linearly from the root to the tip.
%
% 8. Material properties are assumed to be constant throughout
% the structure and correspond to 7075-T6 aluminum.
%
% 9. Euler-Bernoulli beam theory is used, meaning that shear
% deformation effects are neglected and plane sections remain
% plane after bending.
%
% 10. The analysis assumes linear elastic behavior and does not
% account for plastic deformation.
%
% 11. Stress concentrations caused by fasteners, joints,
% cutouts, and manufacturing details are not included in the
% analytical calculations. These effects would be examined
% later using finite element analysis.
%
% 12. Dynamic effects, aeroelastic effects, fatigue, buckling,
% and vibration are outside the scope of this project.
% 
% MATLAB Methods Used:
%   - linspace() → discretization of continuous span
%   - cumtrapz() → numerical integration (trapezoidal rule)
%   - flip() → reverse integration direction (tip → root)
%   - element-wise operators (.* ./ .^) for vectorized computation
%
% Author: Elias Patel
% ========================================================================

clear
clc
close all

% Aircraft Parameters
% ------------------------

% Gravity converts mass into force F = mg
g = 9.81; % m/s^2

% One wing is being analyzed due to symetry.
halfSpan = 6.0; % m

%
grossWeight_lb = 4000; % lb
grossWeight = grossWeight_lb*4.44822; % N

% The 2.5g limit load is increased by a 1.5 Factor of Saftey to obtain our
% ultimate loading condition for our sizing.
limitLoadFactor = 2.5;
ultimateFactor = 1.5;

loadFactor = limitLoadFactor*ultimateFactor;   % 3.75 g

% Lift required to balance aircraft weight under load factor. 
ultimateLift = grossWeight*loadFactor;

% Span discretization for numerical integration for discrete approximation.
N = 1000;
y = linspace(0,halfSpan,N); % m

% Loading Distribution Calculations
% --------------------------------------

% Eliptical Lift Distribution which is derived from Prandtl lifting
% line theory (minimum drag inducing case).
%
% q(y) = q0 * sqrt(1 - (y/L)^2)
q0 = (4*ultimateLift)/(pi*halfSpan); % N/m
liftDistribution = q0*sqrt(1-(y/halfSpan).^2); % N/m

% Wing structual weight with simplified uniform distribution.
wingMass = 150; % kg
wingWeightPerMeter = wingMass*g/halfSpan; % kg/m
wingWeightDistribution = wingWeightPerMeter*ones(size(y)); % N/m

% Fuel weight distribution is approximated as a triangle, which places a
% larger fraction of the fuel load near the fuselage, which is intended as
% a conservative representation as fuel stored preferably inboard, rather
% than sloshing behavior.
fuelMass = 250; % kg
fuelTankLength = 3.0; % m

fuelDistribution = zeros(size(y)); % N/m
fuelDistribution(y <= fuelTankLength) = (fuelMass*g/fuelTankLength) .* (1 - y(y <= fuelTankLength)/fuelTankLength);% N/m

% Net loading will be the total upward force minus the total downward force
% at every vectorized position.
distNet = liftDistribution - wingWeightDistribution - fuelDistribution; % N/m

% Loading Disribution Visualization 

figure

subplot(2, 2, 1);
plot(y,liftDistribution,'LineWidth',2); hold on
plot(y,wingWeightDistribution,'LineWidth',2)
plot(y,fuelDistribution,'LineWidth',2)
plot(y,distNet,'k','LineWidth',3)

grid on
xlabel('Span (m)')
ylabel('Load (N/m)')
title('Spanwise Load Distribution')
legend('Lift','Wing Weight','Fuel','Net')

% Shear Force Calculations and Diagrams
% ------------------------------------------

% Governing shear force equation is an integration of the distributed
% loading. Here we use flip and cumtrapz to perform a trapizoidal
% integration and flip it to go from wing tip to root matching the physical
% condition that shear force at the tip will be zero.
V = flip(cumtrapz(flip(y), flip(distNet))); % N


subplot(2, 2, 2);
plot(y,V/1000,'LineWidth',3)
grid on
xlabel('Span (m)')
ylabel('Shear (kN)')
title('Shear Force Diagram')

% Bending Moment Calculations and Diagrams
% ---------------------------------------------

% Governing bending moment equation is an intergration of shear force.
% Trapizoidal integration is used because we have discrete, not continous
% data points in this simulation.
M = flip(cumtrapz(flip(y), flip(V))); % N*m


subplot(2, 2, 3);
plot(y,M/1000,'LineWidth',3)
grid on
xlabel('Span (m)')
ylabel('Moment (kN·m)')
title('Bending Moment Diagram')

% Loads at Fusalage Attachment 
% ---------------------------------

% Since this is a built in beam, the rooted endpoint is going to experience
% the maximum bending moment, and therefore the maximum shear force.
Vroot = V(1); % N
Mroot = M(1); % N*m

fprintf('\nRoot Shear Force = %.2f kN\n',Vroot/1000);
fprintf('Root Bending Moment = %.2f kN·m\n',Mroot/1000);

% Sectional Geometry and Properties
% --------------------------------------

% Spar dimensions that taper linearly toward the wing tip. Standard in most
% aircraft design.
hRoot = 0.25; hTip = 0.10; % m
bfRoot = 0.08; bfTip = 0.04; % m
tfRoot = 0.012; tfTip = 0.008; % m
twRoot = 0.005; twTip = 0.003; % m

h  = hRoot  - (hRoot-hTip)*(y/halfSpan); % m
bf = bfRoot - (bfRoot-bfTip)*(y/halfSpan); % m
tf = tfRoot - (tfRoot-tfTip)*(y/halfSpan); % m
tw = twRoot - (twRoot-twTip)*(y/halfSpan); % m

% Geometric Feasibility Check
% --------------------------------
% Flanges must fit within the total section height. If 2*tf >= h at any
% station, the web height is zero or negative, which is physically impossible.
if any(2*tf >= h)
    error('Geometry violation: flange thickness exceeds section height at one or more stations. Reduce tf or increase h.');
end

% Web must have positive thickness and flanges must not overlap the web.
if any(tw <= 0) || any(bf <= 0) || any(tf <= 0) || any(h <= 0)
    error('Geometry violation: one or more section dimensions is zero or negative.');
end

% Flange width should be at least as wide as the web thickness (manufacturing minimum).
if any(bf < tw)
    error('Geometry violation: flange width is narrower than web thickness.');
end

% These are properties derived from Mechanics of Meterials (I = Moment of
% Inertia, Q = First Moment of Area, c = Extreme Fiber Distance). They
% change as a function of Geometry.
I = zeros(size(y)); % m^4
Q = zeros(size(y)); % m^3
c = zeros(size(y)); % m

for i = 1:length(y)
    webHeight = h(i) - 2*tf(i);
    flangeArea = bf(i)*tf(i);
    d = webHeight/2 + tf(i)/2; % Distance from the neutral axis.
    
    % Moment of inertia is calculated using parallel axis theorum and the
    % sum of web and flange inertial contributions.
    Iweb = (tw(i)*(webHeight^3))/12;

    Iflange = 2*((bf(i)*tf(i)^3)/12 + flangeArea*d^2);

    I(i) = Iweb + Iflange;

    % The largest shear stress occurs in the middle of the web, so the first
    % moment of area is calculated at the neutral axis.
    Q(i) = flangeArea*d + tw(i)*(webHeight/2)*(webHeight/4);
    c(i) = h(i)/2;
end

% Stress Analysis and Plots
% ------------------------------

% These two equations are the governing equations for bending and shear
% stress.
sigma = abs(M).*c./I; % Pa
tauMaxWeb = abs(V).*Q./(I.*tw); % Pa


subplot(2, 2, 4);
plot(y,sigma/1e6,'LineWidth',3); hold on
plot(y,tauMaxWeb/1e6,'LineWidth',3)

grid on
xlabel('Span (m)')
ylabel('Stress (MPa)')
title('Stress Distribution')
legend('Bending Stress','Maximum Web Shear Stress')

% Spar Deflection and Plot
% -----------------------------

% Assuming we are using Aluminium, these constants are material properties.
E = 71.7e9; % Pa

% We use the Euler-Bernoulli beam equation where EI * d2v/dx2 = M(x). We
% use (in all instances in this model) . before a opperator to signal that
% we are performing this opperation to each individual point in a vector
% (dot product).

curvature = M./(E.*I);

% Integration of slope and deflection.
theta = cumtrapz(y,curvature);
deflection = cumtrapz(y,theta);

figure
plot(y,deflection*1000,'LineWidth',3)

grid on
xlabel('Span (m)')
ylabel('Deflection (mm)')
title('Wing Deflection')

% Final Summary
% ------------------

% Finds our maximum bending moment based on our calculations and our factor
% of saftey based on this max.
sigmaMax = max(sigma);
tauMax = max(tauMaxWeb);

% Yield Strength is used because the spar is designed to remain elastic and
% avoid permanent deformation.
yeildStrengthStress = 434e6; % Pa
FOS_Stress = yeildStrengthStress/sigmaMax;

yeildStrengthShear = 262e6; % Pa
FOS_Shear = yeildStrengthShear/tauMax;

fprintf('\nMax Normal Stress = %.2f MPa\n',sigmaMax/1e6);
fprintf('Max Shear Stress = %.2f MPa\n', tauMax/1e6);
fprintf('\nFOS Normal Stress = %.2f\n', FOS_Stress);
fprintf('FOS Shear Stress = %.2f\n', FOS_Shear);

fprintf('\nRoot Height = %.1f mm\n',h(1)*1000);
fprintf('Root Web Thickness = %.1f mm\n',tw(1)*1000);