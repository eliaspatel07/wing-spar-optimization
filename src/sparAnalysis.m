function [sigma, tau, deflection, volume] = sparAnalysis(x,y, M, V)
    
    % Standardizing to Aluminum
    E = 71.7e9;

    halfSpan = y(end);

    hRoot = x(1);
    hTip = x(2);

    bfRoot = x(3);
    bfTip = x(4);

    tfRoot = x(5); 
    tfTip = x(6);

    twRoot = x(7); 
    twTip = x(8);

    % Geometry
    h  = hRoot  - (hRoot-hTip)*(y/halfSpan); % m
    bf = bfRoot - (bfRoot-bfTip)*(y/halfSpan); % m
    tf = tfRoot - (tfRoot-tfTip)*(y/halfSpan); % m
    tw = twRoot - (twRoot-twTip)*(y/halfSpan); % m

    % Geometric Propeties
    I = zeros(size(y)); % m^4
    Q = zeros(size(y)); % m^3
    c = zeros(size(y)); % m

    for i = 1:length(y)
        webHeight = h(i) - 2*tf(i);
        flangeArea = bf(i)*tf(i);
        d = webHeight/2 + tf(i)/2; 

        Iweb = (tw(i)*(webHeight^3))/12;

        Iflange = 2*((bf(i)*tf(i)^3)/12 + flangeArea*d^2);

        I(i) = Iweb + Iflange;
        Q(i) = flangeArea*d + tw(i)*(webHeight/2)*(webHeight/4);
        c(i) = h(i)/2;
    end
    
    % Stresses

    sigma = abs(M).*c./I; % Pa
    tau = abs(V).*Q./(I.*tw); % Pa\

    % Deflection
    
    curvature = M./(E.*I);
    theta = cumtrapz(y,curvature);
    deflection = cumtrapz(y,theta);

    % Spar Volume

    A = 2*(bf.*tf) + tw.*(h - 2*tf);
    volume = trapz(y, A);

end