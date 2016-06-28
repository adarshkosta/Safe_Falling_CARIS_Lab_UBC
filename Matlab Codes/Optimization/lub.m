%%% Defining the upper and lower bounds of the design variables (states)
function [Y_lb,Y_ub] = lub
global var_array_length

Y_lb = zeros(var_array_length,8);
Y_ub = zeros(var_array_length,8);

for i = 1:var_array_length
    Y_lb(i,1) = pi/6; % Lower bound - ankle joint angle
    Y_ub(i,1) = 2*pi; % Upper bound - ankle joint angle
    Y_lb(i,2) = 5*pi/180; % Lower bound - knee joint angle
    Y_ub(i,2) = 140*pi/180; % Upper bound - knee joint angle
    Y_lb(i,3) = -pi; % Lower bound - ankle joint angular velocity
    Y_ub(i,3) = 0; % Upper bound - ankle joint angular velocity
    Y_lb(i,4) = -10; % Lower bound - knee joint angular velocity
    Y_ub(i,4) = 10; % Upper bound - knee joint angular velocity
    Y_lb(i,5) = -10; % Lower bound - input ankle joint
    Y_ub(i,5) = 10; % Upper bound - input ankle joint
    Y_lb(i,6) = -10; % Lower bound - input knee joint
    Y_ub(i,6) = 10; % Upper bound - input knee joint  
    Y_lb(i,7) = -350/2; % Lower bound - input knee joint
    Y_ub(i,7) = 150/2; % Upper bound - input knee joint
    Y_lb(i,8) = -130/2; % Lower bound - input hip joint
    Y_ub(i,8) = 250/2; % Upper bound - input hip joint
end

Y_lb = Y_lb(:);
Y_ub = Y_ub(:);

end