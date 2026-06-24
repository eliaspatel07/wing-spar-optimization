%% =======================================================================
% Wing Spar Optimization Script
% ========================================================================
%
% Purpose: This script performs constrained minimum-weight optimization of a
% tapered I-beam wing spar under FAR-style 3.75g ultimate maneuver loading.
% Spanwise shear and bending moment distributions are computed from an
% elliptical lift model and integrated numerically using the trapezoidal
% rule. fmincon (SQP algorithm) minimizes spar weight subject to bending
% stress, shear stress, tip deflection, geometric feasibility, and taper
% ratio constraints. The analytical framework is grounded in Euler-Bernoulli
% beam theory and section property formulations from MECHENG 211:
% Introduction to Solid Mechanics, University of Michigan.
%
% Author: Elias Patel
% ========================================================================

clear
clc
close all

%% Loading Calculations

g = 9.81; % m/s^2

halfSpan = 6.0; % m

grossWeight_lb = 4000; % lb
grossWeight = grossWeight_lb*4.44822; % N

limitLoadFactor = 2.5;
ultimateFactor = 1.5;

loadFactor = limitLoadFactor*ultimateFactor;   % 3.75 g

ultimateLift = grossWeight*loadFactor;

N = 1000;
y = linspace(0,halfSpan,N); % m

q0 = (4*ultimateLift)/(pi*halfSpan); % N/m
liftDistribution = q0*sqrt(1-(y/halfSpan).^2); % N/m

wingMass = 150; % kg
wingWeightPerMeter = wingMass*g/halfSpan; % kg/m
wingWeightDistribution = wingWeightPerMeter*ones(size(y)); % N/m

fuelMass = 250; % kg
fuelTankLength = 3.0; % m

fuelDistribution = zeros(size(y)); % N/m
fuelDistribution(y <= fuelTankLength) = (fuelMass*g/fuelTankLength) .* (1 - y(y <= fuelTankLength)/fuelTankLength); % N/m

distNet = liftDistribution - wingWeightDistribution - fuelDistribution; % N/m

V = flip(cumtrapz(flip(y), flip(distNet))); % N
M = flip(cumtrapz(flip(y), flip(V))); % N*m

%% Optimization Bounds

lowerBound = [0.15, 0.05, 0.04, 0.02, 0.003, 0.003, 0.002, 0.002];
upperBound = [0.80, 0.35, 0.15, 0.10, 0.030, 0.020, 0.015, 0.010];

%% Optimization Settings

options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter');

nStarts = 10;

bestWeight = inf;

%% Multi-Start Optimization

for i = 1:nStarts

    x0 = lowerBound + rand(1,8).*(upperBound - lowerBound);

    % Run optimization for the current starting point
    [xOpt, fval, exitflag] = fmincon(@(x) objectiveFunction(x, y, M, V), x0, [], [], [], [], lowerBound, upperBound, @(x) nonlinearConstraints(x, y, M, V), options);

    % Update best weight if the current optimization yields a better result
    if exitflag > 0
        
        if fval < bestWeight
            bestWeight = fval;
            bestX = xOpt;
        end
    
    end

end

%% Report the Optimized Results

fprintf('\nOptimized Spar Weight = %.2f kg\n', bestWeight);

% Optimized Design Variables are printed in two different units of
% measurement in order to be inputed into Parametric Modeling Programs.
disp('Optimized Design Variables (m): ')
disp(bestX);

fprintf('\nOptimized Design Variables (in): \n')
bestXInches = bestX .* 39.3701;
disp(bestXInches)

%% Determine Limiting Factor

[c, ~] = nonlinearConstraints(bestX, y, M, V);

disp('Constraint Values: ');
disp(c);

%% Validation Check

[sigma,tau,deflection,volume] = sparAnalysis(bestX,y,M,V);

fprintf('\nValidation Results:\n');

fprintf('Max Bending Stress = %.2f MPa\n', max(sigma)/1e6);

fprintf('Max Web Shear Stress = %.2f MPa\n', max(tau)/1e6);

fprintf('Tip Deflection = %.2f mm\n', max(abs(deflection))*1000);

fprintf('Spar Volume = %.5f m^3\n', volume);

%% Factor of Saftey and Other Useful Information

yieldStress = 434e6; % Pa
yieldShear = 262e6; % Pa
tipLimit = 0.30; % m

FOS_Bending = yieldStress/max(sigma);

FOS_Shear = yieldShear/max(tau);

FOS_Deflection = tipLimit/max(abs(deflection));

fprintf('\nFactors of Safety\n');

fprintf('Normal Stress FOS = %.2f\n', FOS_Bending);

fprintf('Shear Stress FOS = %.2f\n', FOS_Shear);

fprintf('Deflection FOS = %.2f\n', FOS_Deflection);
