%%% Generating an array of the initial guess, including all the design
%%% varibales. Governing equations of motion for a triple link inverted
%%% pendulum are discretized and solved using forward differencing method.
%%% A = [a11 a12 a13 ; a21 a22 a23 ; a31 a32 a33]
%%% C = [c11 ; c21 ; c31]
%%% A*ddteta = C and D = inv(A)*C
%%% teta = [teta_1 ; teta_2 ; teta_3 ; dteta_1 ; dteta_2 ; dteta_3]
%%% dteta = [dteta_1 ; dteta_2 ; dteta_3 ; ddteta_1 ; ddteta_2 ; ddteta_3]
%%% dteta = [teta(4) ; teta(5) ; teta(6) ; D(1) ; D(2) ; D(3)]
function [Y_0,X_0] = ini_guess
global N J_rotor L0 L1 L2 L3 M1 M2 M3 rCOM_1 rCOM_2 rCOM_3 g rG_1 rG_2 rG_3 dt Torque_2 Torque_3 teta_01 teta_02 teta_03 dteta_01 dteta_02 dteta_03 var_array_length

X_0(1,1) = teta_01;  % Ankle angle - Initial condition
X_0(1,2) = teta_02;  % Knee angle - Initial condition
X_0(1,3) = teta_03;  % Hip angle - Initial condition
X_0(1,4) = dteta_01; % Ankle angular velocity - Initial condition
X_0(1,5) = dteta_02; % Knee angular velocity - Initial condition
X_0(1,6) = dteta_03; % Hip angular velocity - Initial condition
X_0(1,7) = Torque_2; % Applied torque at the knee joint to generate the initial guess
X_0(1,8) = Torque_3; % Applied torque at the hip joint to generate the initial guess


A = ones(var_array_length,3,3);
C = ones(var_array_length,3,1);



for i = 2:var_array_length % Generating the initial guess using forward differencing method
    a11 = M1*rG_1^2*L1^2+M2*(rG_2^2*L2^2+(L0+L1)^2+2*rCOM_2*(L0+L1)*L2*cos(X_0(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+(L0+L1)^2+2*rCOM_3*L3*L2*cos(X_0(i-1,3))+2*L2*(L0+L1)*cos(X_0(i-1,2))+2*rCOM_3*(L0+L1)*L3*cos(X_0(i-1,2)+X_0(i-1,3)));
    a12 = M2*(rG_2^2*L2^2+rCOM_2*(L0+L1)*L2*cos(X_0(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L3*L2*cos(X_0(i-1,3))+L2*(L0+L1)*cos(X_0(i-1,2))+rCOM_3*(L0+L1)*L3*cos(X_0(i-1,2)+X_0(i-1,3)));
    a13 =  M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X_0(i-1,3))+rCOM_3*(L0+L1)*L3*cos(X_0(i-1,2)+X_0(i-1,3)));
    c11 = M3*(2*rCOM_3*L2*L3*sin(X_0(i-1,3))*X_0(i-1,6)+2*L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,5)+2*rCOM_3*(L0+L1)*L3*sin(X_0(i-1,2)+X_0(i-1,3))*(X_0(i-1,5)+X_0(i-1,6)))*X_0(i-1,4)...
        +M3*(2*rCOM_3*L2*L3*sin(X_0(i-1,3))*X_0(i-1,6)+L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,5)+rCOM_3*(L0+L1)*L3*sin(X_0(i-1,2)+X_0(i-1,3))*(X_0(i-1,5)+X_0(i-1,6)))*X_0(i-1,5)...
        +M3*(rCOM_3*L2*L3*sin(X_0(i-1,3))*X_0(i-1,6)+rCOM_3*(L0+L1)*L3*sin(X_0(i-1,2)+X_0(i-1,3))*(X_0(i-1,5)+X_0(i-1,6)))*X_0(i-1,6)...
        +M2*(2*rCOM_2*L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,4)*X_0(i-1,5)+rCOM_2*(L0+L1)*L2*sin(X_0(i-1,2))*(X_0(i-1,5))^2)...
        -rCOM_1*M1*L1*g*cos(X_0(i-1,1))-(M2+M3)*(L0+L1)*g*cos(X_0(i-1,1))-(rCOM_2*M2+M3)*L2*g*cos(X_0(i-1,1)+X_0(i-1,2))-rCOM_3*M3*g*L3*cos(X_0(i-1,1)+X_0(i-1,2)+X_0(i-1,3));

    a21 = M2*(rG_2^2*L2^2+rCOM_2*(L0+L1)*L2*cos(X_0(i-1,2)))...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L2*L3*cos(X_0(i-1,3))+(L0+L1)*L2*cos(X_0(i-1,2))+rCOM_3*(L0+L1)*L3*cos(X_0(i-1,2)+X_0(i-1,3)));
    a22 = rG_2^2*M2*L2^2 ...
        +M3*(rG_3^2*L3^2+L2^2+2*rCOM_3*L2*L3*cos(X_0(i-1,3)))+J_rotor*N^2;
    a23 = M3*(rG_3^2*L3^2+rCOM_3*L2*L3*cos(X_0(i-1,3)));
    c21 = M3*(2*rCOM_3*L3*L2*sin(X_0(i-1,3))*X_0(i-1,6)+L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,5)+rCOM_3*(L0+L1)*L3*sin(X_0(i-1,2)+X_0(i-1,3))*(X_0(i-1,5)+X_0(i-1,6)))*X_0(i-1,4)...
        +M3*2*rCOM_3*L3*L2*sin(X_0(i-1,3))*X_0(i-1,6)*X_0(i-1,5)...
        +M3*rCOM_3*L3*L2*sin(X_0(i-1,3))*X_0(i-1,6)*X_0(i-1,6)...
        +M2*rCOM_2*L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,4)*X_0(i-1,5)...
        -M3*(((L0+L1)*L2*sin(X_0(i-1,2))+rCOM_3*(L0+L1)*L3*sin(X_0(i-1,2)+X_0(i-1,3)))*(X_0(i-1,5)+X_0(i-1,4))+(rCOM_3*L3*(L0+L1)*sin(X_0(i-1,2)+X_0(i-1,3))*X_0(i-1,6)))*X_0(i-1,4)...
        -M2*rCOM_2*L2*(L0+L1)*sin(X_0(i-1,2))*X_0(i-1,4)*(X_0(i-1,4)+X_0(i-1,5))...
        -(rCOM_2*M2+M3)*L2*g*cos(X_0(i-1,1)+X_0(i-1,2))-(rCOM_3*M3)*L3*g*cos(X_0(i-1,1)+X_0(i-1,2)+X_0(i-1,3))...
        +X_0(i-1,7);

    a31 = M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X_0(i-1,3))+rCOM_3*L3*(L0+L1)*cos(X_0(i-1,2)+X_0(i-1,3)));
    a32 = M3*(rG_3^2*L3^2+rCOM_3*L3*L2*cos(X_0(i-1,3)));
    a33 = M3*rG_3^2*L3^2+J_rotor*N^2;
    c31 = M3*(rCOM_3*L3*L2*sin(X_0(i-1,3))*X_0(i-1,6)+rCOM_3*L3*(L0+L1)*sin(X_0(i-1,2)+X_0(i-1,3))*(X_0(i-1,5)+X_0(i-1,6)))*X_0(i-1,4)...
        +M3*(rCOM_3*L3*L2*sin(X_0(i-1,3))*X_0(i-1,6))*X_0(i-1,5)...
        -M3*rCOM_3*L3*(X_0(i-1,4)+X_0(i-1,5)+X_0(i-1,6))*(X_0(i-1,4)*(L2*sin(X_0(i-1,3))+(L0+L1)*sin(X_0(i-1,2)+X_0(i-1,3)))+L2*X_0(i-1,5)*sin(X_0(i-1,3)))...
        -rCOM_3*M3*g*L3*cos(X_0(i-1,1)+X_0(i-1,2)+X_0(i-1,3))...
        +X_0(i-1,8);

    
    
    D = ([a11 a12 a13 ; a21 a22 a23 ; a31 a32 a33])\[c11 ; c21 ; c31];
    X_0(i,1) = dt*X_0(i-1,4)+ X_0(i-1,1); 
    X_0(i,2) = dt*X_0(i-1,5)+ X_0(i-1,2); 
    X_0(i,3) = dt*X_0(i-1,6)+ X_0(i-1,3); 
    X_0(i,4) = dt*D(1)+ X_0(i-1,4);
    X_0(i,5) = dt*D(2)+ X_0(i-1,5);
    X_0(i,6) = dt*D(3)+ X_0(i-1,6);
    X_0(i,7) = Torque_2;
    X_0(i,8) = Torque_3;
end

Y_0 = X_0(:); % Initial guess in array format

end