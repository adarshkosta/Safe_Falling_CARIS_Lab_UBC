%% Defining nonlinear equality and inequality constraints
function [c,Y_ceq] = NonLin_Cons(Y)
global N J_rotor L0 L1 L2 L3 M1 M2 M3 rCOM_1 rCOM_2 rCOM_3 g rG_1 rG_2 rG_3 dt teta_01 teta_02 teta_03 dteta_01 dteta_02 dteta_03 var_array_length
X(:,1) = Y(1:var_array_length);
X(:,2) = Y(var_array_length+1:2*var_array_length);
X(:,3) = Y(2*var_array_length+1:3*var_array_length);
X(:,4) = Y(3*var_array_length+1:4*var_array_length);
X(:,5) = Y(4*var_array_length+1:5*var_array_length);
X(:,6) = Y(5*var_array_length+1:6*var_array_length);
X(:,7) = Y(6*var_array_length+1:7*var_array_length);
X(:,8) = Y(7*var_array_length+1:8*var_array_length);

c(1) = (L0+L1)*sin(X(end,1))+L2*sin(X(end,1)+X(end,2)); % Nonlinear inequality constraint - defining the moment of impact
c(2) = -((L0+L1)*sin(X(end-1,1))+L2*sin(X(end-1,1)+X(end-1,2))); % Nonlinear inequality constraint - defining the moment of impact

ceq(1,1) = X(1,1)-teta_01; % Nonlinear eqaulity constraint - Defining the initial condition
ceq(1,2) = X(1,2)-teta_02; % Nonlinear eqaulity constraint - Defining the initial condition
ceq(1,3) = X(1,3)-teta_03; % Nonlinear eqaulity constraint - Defining the initial condition
ceq(1,4) = X(1,4)-dteta_01; % Nonlinear eqaulity constraint - Defining the initial condition
ceq(1,5) = X(1,5)-dteta_02; % Nonlinear eqaulity constraint - Defining the initial condition
ceq(1,6) = X(1,6)-dteta_03; % Nonlinear eqaulity constraint - Defining the initial condition

for i = 2:var_array_length % Generating nonlinear equality constraints
    a11 = M1*rG_1^2*L1^2+M2*(rG_2^2*L2^2+(L0+L1)^2+2*rCOM_2*(L0+L1)*L2*cos(X(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+(L0+L1)^2+2*rCOM_3*L3*L2*cos(X(i-1,3))+2*L2*(L0+L1)*cos(X(i-1,2))+2*rCOM_3*(L0+L1)*L3*cos(X(i-1,2)+X(i-1,3)));
    a12 = M2*(rG_2^2*L2^2+rCOM_2*(L0+L1)*L2*cos(X(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L3*L2*cos(X(i-1,3))+L2*(L0+L1)*cos(X(i-1,2))+rCOM_3*(L0+L1)*L3*cos(X(i-1,2)+X(i-1,3)));
    a13 =  M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X(i-1,3))+rCOM_3*(L0+L1)*L3*cos(X(i-1,2)+X(i-1,3)));
    c11 = M3*(2*rCOM_3*L2*L3*sin(X(i-1,3))*X(i-1,6)+2*L2*(L0+L1)*sin(X(i-1,2))*X(i-1,5)+2*rCOM_3*(L0+L1)*L3*sin(X(i-1,2)+X(i-1,3))*(X(i-1,5)+X(i-1,6)))*X(i-1,4)...
        +M3*(2*rCOM_3*L2*L3*sin(X(i-1,3))*X(i-1,6)+L2*(L0+L1)*sin(X(i-1,2))*X(i-1,5)+rCOM_3*(L0+L1)*L3*sin(X(i-1,2)+X(i-1,3))*(X(i-1,5)+X(i-1,6)))*X(i-1,5)...
        +M3*(rCOM_3*L2*L3*sin(X(i-1,3))*X(i-1,6)+rCOM_3*(L0+L1)*L3*sin(X(i-1,2)+X(i-1,3))*(X(i-1,5)+X(i-1,6)))*X(i-1,6)...
        +M2*(2*rCOM_2*L2*(L0+L1)*sin(X(i-1,2))*X(i-1,4)*X(i-1,5)+rCOM_2*(L0+L1)*L2*sin(X(i-1,2))*(X(i-1,5))^2)...
        -rCOM_1*M1*L1*g*cos(X(i-1,1))-(M2+M3)*(L0+L1)*g*cos(X(i-1,1))-(rCOM_2*M2+M3)*L2*g*cos(X(i-1,1)+X(i-1,2))-rCOM_3*M3*g*L3*cos(X(i-1,1)+X(i-1,2)+X(i-1,3);

    a21 = M2*(rG_2^2*L2^2+rCOM_2*(L0+L1)*L2*cos(X(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L2*L3*cos(X(i-1,3))+(L0+L1)*L2*cos(X(i-1,2))+rCOM_3*(L0+L1)*L3*cos(X(i-1,2)+X(i-1,3)));
    a22 = rG_2^2*M2*L2^2 ...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L2*L3*cos(X(i-1,3)))+ J_rotor*N^2;
    a23 = M3*(rG_3^2*L3^2+rCOM_3*L2*L3*cos(X(i-1,3)));
    c21 = M3*(2*rCOM_3*L3*L2*sin(X(i-1,3))*X(i-1,6)+L2*(L0+L1)*sin(X(i-1,2))*X(i-1,5)+rCOM_3*(L0+L1)*L3*sin(X(i-1,2)+X(i-1,3))*(X(i-1,5)+X(i-1,6)))*X(i-1,4)...
        +M3*2*rCOM_3*L3*L2*sin(X(i-1,3))*X(i-1,6)*X(i-1,5)...
        +M3*rCOM_3*L3*L2*sin(X(i-1,3))*X(i-1,6)*X(i-1,6)...
        +M2*rCOM_2*L2*(L0+L1)*sin(X(i-1,2))*X(i-1,4)*X(i-1,5)...
        -M3*(((L0+L1)*L2*sin(X(i-1,2))+rCOM_3*(L0+L1)*L3*sin(X(i-1,2)+X(i-1,3)))*(X(i-1,5)+X(i-1,4))+(rCOM_3*L3*(L0+L1)*sin(X(i-1,2)+X(i-1,3))*X(i-1,6)))*X(i-1,4)...
        -M2*rCOM_2*L2*(L0+L1)*sin(X(i-1,2))*X(i-1,4)*(X(i-1,4)+X(i-1,5))...
        -(rCOM_2*M2+M3)*L2*g*cos(X(i-1,1)+X(i-1,2))-(rCOM_3*M3)*L3*g*cos(X(i-1,1)+X(i-1,2)+X(i-1,3))...
        +X(i-1,7);

    a31 = M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X(i-1,3))+rCOM_3*L3*(L0+L1)*cos(X(i-1,2)+X(i-1,3)));
    a32 = M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X(i-1,3)));
    a33 = M3*rG_3^2*L3^2+ J_rotor*N^2;
    c31 = M3*(rCOM_3*L3*L2*sin(X(i-1,3))*X(i-1,6)+rCOM_3*L3*(L0+L1)*sin(X(i-1,2)+X(i-1,3))*(X(i-1,5)+X(i-1,6)))*X(i-1,4)...
        +M3*(rCOM_3*L3*L2*sin(X(i-1,3))*X(i-1,6))*X(i-1,5)...
        -M3*rCOM_3*L3*(X(i-1,4)+X(i-1,5)+X(i-1,6))*(X(i-1,4)*(L2*sin(X(i-1,3))+(L0+L1)*sin(X(i-1,2)+X(i-1,3)))+L2*X(i-1,5)*sin(X(i-1,3)))...
        -rCOM_3*M3*g*L3*cos(X(i-1,1)+X(i-1,2)+X(i-1,3))...
        +X(i-1,8);

    D = ([a11 a12 a13 ; a21 a22 a23 ; a31 a32 a33])\[c11 ; c21 ; c31];
    ceq(i,1) = X(i,1) - (dt*X(i-1,4)+ X(i-1,1));
    ceq(i,2) = X(i,2) - (dt*X(i-1,5)+ X(i-1,2)); 
    ceq(i,3) = X(i,3) - (dt*X(i-1,6)+ X(i-1,3)); 
    ceq(i,4) = X(i,4) - (dt*D(1)+ X(i-1,4));
    ceq(i,5) = X(i,5) - (dt*D(2)+ X(i-1,5));
    ceq(i,6) = X(i,6) - (dt*D(3)+ X(i-1,6));
end

Y_ceq = ceq(:);

end