function [space1, N1,error1, space2, N2,error2, space3, N3,error3,error_tot,error_M1,error_M2,error_M3] = optimize_link_configuration_optimize(L1,L2,L3)
%links contains a list of links to optimize


global M1_target M2_target M3_target N1_max N2_max N3_max rCOM_1_target ...
    rCOM_2_target rCOM_3_target rG_1_target rG_2_target rG_3_target L1_opt L2_opt L3_opt
%% Target Mass properties

rCOM_1_target = 0.394; % Distal distance of center of mass of the foot and shank segment given as a percent of shank length
rCOM_2_target = 0.567; % Distal distance of center of mass of the thigh segment given as a percent of thigh length
rCOM_3_target = 0.626; % Proximal distance of center of mass of the HAT (Head, Arms, Trunk) segment given as a percent of trunk length
rG_1_target = 0.572; % Distal radius of gyration of the foot and shank segment given as a percent of shank length
rG_2_target = 0.653; % Distal radius of gyration of the thigh segment given as a percent of thigh length
rG_3_target = 0.798; % Proximal radius of gyration of HAT segment given as percent of trunk length

M1_target = 0.061*2; % Foot and shank mass
M2_target = 0.1*2; % Thigh mass
M3_target = 0.678; % Half the mass of the HAT segment

L1_opt = L1;
L2_opt = L2;
L3_opt = L3;
%% Optimization

w = 1.5e-3*5; %washer width

N1_max = floor(L1/w); %maximumum number of washers that can be added to shank
N2_max = floor(L2/w); %maximumum number of washers that can be added to thigh
N3_max = floor(L3/w); %maximumum number of washers that can be added to trunk


rCOM_1_best = 0;
rCOM_2_best = 0;
rCOM_3_best = 0;
rG_1_best = 0;
rG_2_best = 0;
rG_3_best = 0;

configureModel(L1,L2,L3,0,0,0,0,0,0); %initialize global variables
%% Optimal parameters

lb = [0,0,0,0,0,0];
ub = [N1_max,N1_max,N2_max,N2_max,N3_max,N3_max];
fun = @objective_function_mass_opt;
nonlcon = @nonlinearcons_mass_opt;
x0 = 2*[28,0,14,12,27,23];
x = fmincon(fun,x0,[],[],[],[],lb,ub,nonlcon);

N1 = x(1)*5;
N2 = x(3)*5;
N3 = x(5)*5;
skip1 = x(2);
skip2 = x(4);
skip3 = x(6);
%x(1) = N1
%x(2) = skip1
%x(3) = N2
%x(4) = skip2
%x(5) = N3
%x(6) = skip3

space1 = skip1*w;


space2 = skip2*w;


space3 = skip3*w;

[M1,M2,M3,rCOM_1,rCOM_2,rCOM_3,rG_1,rG_2,rG_3] = configureModel(L1_opt,L2_opt,L3_opt,x(1),x(3),x(5),x(2),x(4),x(6));
     
if(N2 < 0)
    x(3) = 0;
    N2 = 0;
    [M1,M2,M3,rCOM_1,rCOM_2,rCOM_3,rG_1,rG_2,rG_3] = configureModel(L1_opt,L2_opt,L3_opt,x(1),x(3),x(5),x(2),x(4),x(6));
end
error1 = sqrt((abs(rCOM_1-rCOM_1_target)/rCOM_1_target)^2 + (abs(rG_1-rG_1_target)/rG_1_target)^2);
 
error2 = sqrt((abs(rCOM_2-rCOM_2_target)/rCOM_2_target)^2 + (abs(rG_2-rG_2_target)/rG_2_target)^2);
 
error3 = sqrt((abs(rCOM_3-rCOM_3_target)/rCOM_3_target)^2 + (abs(rG_3-rG_3_target)/rG_3_target)^2);

M_tot = M1+M2+M3;
error_M1 = abs(M1/M_tot - M1_target)/M1_target
error_M2 = abs(M2/M_tot - M2_target)/M2_target
error_M3 = abs(M3/M_tot - M3_target)/M3_target
error_tot = sqrt(error1^2+error2^2+error3^2)

end