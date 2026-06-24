function weight = objectiveFunction(x, y, M, V)
    
    density = 2810;
    
    [~, ~, ~, volume] = sparAnalysis(x, y, M, V);

    weight = density * volume;

end