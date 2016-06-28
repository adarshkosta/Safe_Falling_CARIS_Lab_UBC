function [J_adj,com_adj,M_adj] = adjustable_mass_properties(link,N,skip)
global m
Ry_link = 0.001*[19.65 15.636 21.815];
Ry = Ry_link(link); %displacement between adjustable mass and reference axis (perpendicular to link axis)
%Rx_link = 0.001*[71.843 62.01 175.36];
Rx_link = [0 0 0];
Rx = Rx_link(link); %displacement between start of adjustable mass and reference axis (parralel to link axis)
w = 0.0015*5; %washer thinkness
r1 = 0.0032; %washer IR
r2 = 0.009; %washer OR
m = 0.00225*5; %washer mass

R = sqrt(Ry^2+(Rx+skip*w+N/2*w)^2); %total displacement between center of adjustable mass and reference axis
J_adj = m*N*(3*(r1^2+r1^2)+(N*w)^2)/12; %total inertia of adjustable mass mechanism from reference axis

M_adj = N*m; %total mass of adjustable mass mechanism
J_adj = J_adj + M_adj*R^2;
com_adj = Rx+skip*w+N/2*w; %center of mass of adustable mass mechanism projected onto link axis

end