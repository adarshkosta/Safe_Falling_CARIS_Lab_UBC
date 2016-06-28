function y = objective_function_mass_opt(x)

global weight M1_no_mass M2_no_mass M3_no_mass m M1_target M2_target M3_target  rCOM_1_target  rCOM_2_target rCOM_3_target rG_1_target rG_2_target rG_3_target L1_opt L2_opt L3_opt


[M1,M2,M3,rCOM_1,rCOM_2,rCOM_3,rG_1,rG_2,rG_3] = configureModel(L1_opt,L2_opt,L3_opt,x(1),x(3),x(5),x(2),x(4),x(6));
        
error1 = sqrt((abs(rCOM_1-rCOM_1_target)/rCOM_1_target)^2 + (abs(rG_1-rG_1_target)/rG_1_target)^2);
 
error2 = sqrt((abs(rCOM_2-rCOM_2_target)/rCOM_2_target)^2 + (abs(rG_2-rG_2_target)/rG_2_target)^2);
 
error3 = sqrt((abs(rCOM_3-rCOM_3_target)/rCOM_3_target)^2 + (abs(rG_3-rG_3_target)/rG_3_target)^2);

% error1 = sqrt((abs(rCOM_1-rCOM_1_target)/rCOM_1_target)^2 );
%  
% error2 = sqrt((abs(rCOM_2-rCOM_2_target)/rCOM_2_target)^2 );
%  
% error3 = sqrt((abs(rCOM_3-rCOM_3_target)/rCOM_3_target)^2 );
% 
M_tot = M1+M2+M3;
error1M = abs(M1/M_tot - M1_target)/M1_target;
error2M  = abs(M2/M_tot - M2_target)/M2_target;
error3M  = abs(M3/M_tot - M3_target)/M3_target;

%+error1M^2+error2M^2+error3M ^2
y = sqrt((weight)*(error1^2 + error2^2 + error3^2) + (1-weight)*(error1M^2+error2M^2+error3M ^2));
end