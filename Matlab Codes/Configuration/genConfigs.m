function configs = genConfigs(N_max)
trials = (N_max+1)*(N_max)/2;

configs = zeros(trials,2);
idx = 1;
for i = 0:N_max
    for j = 0:N_max-i
        configs(idx,1) = i;
        configs(idx,2) = j; 
        idx = idx+1;
    end
end

end
