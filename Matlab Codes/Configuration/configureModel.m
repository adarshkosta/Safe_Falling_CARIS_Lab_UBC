function [M1_tot,M2_tot,M3_tot,rCOM_1,rCOM_2,rCOM_3,rG_1,rG_2,rG_3] = configureModel(L1,L2,L3,N1,N2,N3,skip1,skip2,skip3)

global M1_no_mass M2_no_mass M3_no_mass 
%L1 = total shank length
%L2 = total thigh length
%L3 = total trunk length

%% Mass scaling factor

M =  1.5390; %total measured mass of 3-link system
M_cad =  1.3400; %total mass of 3-link system from mass properties

f_c = M/M_cad; %mass scaling factor between real and cad models

%% CAD model mass properties


shank_1_M = 0.1326; %shank motor side mass
shank_2_M = 0.06387; %shank expansion side mass
shank_1_com = 83.1173e-3; %distal center of mass of thigh motor side
shank_2_com = 46.1327e-3; %proximal center of mass  of thigh expansion side;
shank_1_J = 1447.63739e-6; %distal mass moment of inertia of thigh motor side
shank_2_J = 112.056e-6; %mass moment of inertia about COM of thigh expansion side;

thigh_1_M = 0.43907; %thigh motor side mass
thigh_2_M = 0.0503; %thigh expansion side mass
thigh_1_com = 67.60191e-3; %distal center of mass of thigh motor side
thigh_2_com =  28.91e-3; %proximal center of mass  of thigh expansion side;
thigh_1_J =  2685.92881e-6; %distal mass moment of inertia of thigh motor side
thigh_2_J = 35.46116e-6; %mass moment of inertia about COM of thigh expansion side;


trunk_1_M = 0.48; %shank motor side mass
trunk_2_M = 0.16031; %shank expansion side mass
trunk_1_com = 85.29e-3; %proximal center of mass of thigh motor side
trunk_2_com = 67.47e-3; %distal center of mass  of thigh expansion side;
trunk_1_J = 4900.44e-6; %distal mass moment of inertia of thigh motor side
trunk_2_J = 504.13e-6; % mass moment of inertia about COM of thigh expansion side;

%% Total mass properties with no added mass
M1 = shank_1_M + shank_2_M; %total shank mass with no added mass
shank_com = (L2-shank_2_com)*shank_2_M/M1 + shank_1_com*shank_1_M/M1; % distal center of mass of shank
shank_J = shank_1_J + (L1-shank_2_com)^2*thigh_2_J; % distal mass moment of inertia of shank

M2 = thigh_1_M + thigh_2_M; %total thigh mass with no added mass
thigh_com = (L2-thigh_2_com)*thigh_2_M/M2 + thigh_1_com*thigh_1_M/M2; % distal center of mass of thigh
thigh_J = thigh_1_J + (L2-thigh_2_com)^2*thigh_2_J; % distal mass moment of inertia of thigh

M3 = trunk_1_M + trunk_2_M; %total trunk mass with no added mass
trunk_com = (L3-trunk_2_com)*trunk_2_M/M3 + trunk_1_com*trunk_1_M/M3; % proximal center of mass of thigh
trunk_J = trunk_1_J + (L3-trunk_2_com)^2*trunk_2_J; % proximal mass moment of inertia of trunk

M1_no_mass = M1;
M2_no_mass = M2;
M3_no_mass = M3;
%% Scale inertias to correct for differences between CAD and real life

M1 = M1*f_c;
M2 = M2*f_c;
M3 = M3*f_c;

shank_J = shank_J*f_c; 
thigh_J = thigh_J*f_c; 
trunk_J = trunk_J*f_c; 

%% Find mass of adjustable mass mechanism 

% N1 = number of washers on shank;
% N2 = number of washers on thigh;
% N3 = number of washers on trunk;
% skip1 = how many washer-width spaces to skip before possitionin the first washer
% skip2 = how many washer-width spaces to skip before possitionin the first washer
% skip3 = how many washer-width spaces to skip before possitionin the first washer
[shank_J_adj, shank_com_adj, M1_adj] = adjustable_mass_properties(1,N1,skip1);
[thigh_J_adj, thigh_com_adj, M2_adj] = adjustable_mass_properties(2,N2,skip2);
[trunk_J_adj, trunk_com_adj, M3_adj] = adjustable_mass_properties(3,N3,skip3);

%% Combine mass properties

shank_J_tot = shank_J+shank_J_adj;
thigh_J_tot = thigh_J+thigh_J_adj;
trunk_J_tot = trunk_J+trunk_J_adj;

M1_tot = M1+M1_adj;
M2_tot = M2+M2_adj;
M3_tot = M3+M3_adj;

shank_com_tot = shank_com*M1/M1_tot + shank_com_adj*M1_adj/M1_tot;
thigh_com_tot = thigh_com*M2/M2_tot + thigh_com_adj*M2_adj/M2_tot;
trunk_com_tot = trunk_com*M3/M3_tot + trunk_com_adj*M3_adj/M3_tot;

rCOM_1 = shank_com_tot/L1; % Distal distance of center of mass of the foot and shank segment given as a percent of shank length
rCOM_2 = thigh_com_tot/L2; % Distal distance of center of mass of the thigh segment given as a percent of thigh length
rCOM_3 = trunk_com_tot/L3; % Proximal distance of center of mass of the HAT (Head, Arms, Trunk) segment given as a percent of trunk length
rG_1 = sqrt(shank_J_tot/M1_tot)/L1; % Distal radius of gyration of the foot and shank segment given as a percent of shank length
rG_2 = sqrt(thigh_J_tot/M2_tot)/L2; % Distal radius of gyration of the thigh segment given as a percent of thigh length
rG_3 = sqrt(trunk_J_tot/M3_tot)/L3; % Proximal radius of gyration of HAT segment given as percent of trunk length

end

