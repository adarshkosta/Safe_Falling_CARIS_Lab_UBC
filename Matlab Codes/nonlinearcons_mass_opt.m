function [ c,ceq ] = nonlinearcons_mass_opt( x )
%x(1) = N1
%x(2) = skip1
%x(3) = N2
%x(4) = skip2
%x(5) = N3
%x(6) = skip3
global M1_no_mass M2_no_mass M3_no_mass m M1_target M2_target M3_target N1_max N2_max N3_max
thresh = 1; %required accuracy of mass distribution
M_no_mass = M1_no_mass+M2_no_mass+M3_no_mass;
M_tot = M_no_mass + (x(1)+x(3)+x(3))*m;
c = zeros(6,1);
%  c(1,1) = abs((x(1)*m+M1_no_mass)/M_tot - M1_target)/M1_target-thresh;
%  c(2,1) = abs((x(3)*m+M2_no_mass)/M_tot - M2_target)/M2_target-thresh;
%  c(3,1) = abs((x(5)*m+M3_no_mass)/M_tot - M3_target)/M3_target-thresh;
c(4,1) = x(1)+x(2)-N1_max;
c(5,1) = x(3)+x(4)-N2_max;
c(6,1) = x(5)+x(6)-N3_max;

ceq = [];
end

