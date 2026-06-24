function [c,ceq] = nonlinearConstraints(x,y,M,V)

    [sigma,tau,deflection,~] = sparAnalysis(x,y,M,V);

    yieldStress = 434e6; % Pa
    yieldShear  = 262e6; % Pa
    
    tipLimit = 0.30; % m

    % Structural constraints

    cStress = max(sigma)/yieldStress - 1;

    cShear = max(tau)/yieldShear - 1;

    cDeflection = max(abs(deflection))/tipLimit - 1;

    halfSpan = y(end);

    h  = x(1) - (x(1)-x(2))*(y/halfSpan);
    bf = x(3) - (x(3)-x(4))*(y/halfSpan);
    tf = x(5) - (x(5)-x(6))*(y/halfSpan);
    tw = x(7) - (x(7)-x(8))*(y/halfSpan);

    % Geometric Constraints
    
    cGeom1 = max(2*tf - h);

    cGeom2 = -min(h);

    cGeom3 = -min(bf);
    cGeom4 = -min(tf);

    cGeom5 = -min(tw);

    cGeom6 = max(tw - bf);

    % Taper Constraints
       
    taperMin_h = 0.35;
    taperMin_bf = 0.40;

    cTaper1 = taperMin_h*x(1) - x(2);    % hTip >= 0.35*hRoot

    cTaper2 = taperMin_bf*x(3) - x(4);   % bfTip >= 0.40*bfRoot;

    % Assemble Constraints

    c = [cStress, cShear, cDeflection, cGeom1, cGeom2, cGeom3, cGeom4, cGeom5, cGeom6, cTaper1, cTaper2];

    ceq = [];

end