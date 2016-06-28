%% Defining the objective function - the vertical component of velocity
function Obj_Fcn_Value = Obj_Fcn(Y)
global L0 L1 L2 var_array_length

X(:,1) = Y(1:var_array_length); % Individual design variable - Ankle angle
X(:,2) = Y(var_array_length+1:2*var_array_length); % Individual design variable - Knee angle
X(:,3) = Y(2*var_array_length+1:3*var_array_length); % Design variable - Hip angle
X(:,4) = Y(3*var_array_length+1:4*var_array_length); % Design variable - Ankle angular velocity
X(:,5) = Y(4*var_array_length+1:5*var_array_length); % Design variable - Knee angular velocity
X(:,6) = Y(5*var_array_length+1:6*var_array_length); % Design variable - Hip angular velocity
X(:,7) = Y(6*var_array_length+1:7*var_array_length); % Design variable - Applied torque at the knee joint
X(:,8) = Y(7*var_array_length+1:8*var_array_length); % Design variable - Applied torque at the hip joint

Obj_Fcn_Value = (L2.*(X(end-1,4)+X(end-1,5))).^2+((L0+L1).*X(end-1,4)).^2+2*L2*(L0+L1).*cos(X(end-1,2)).*X(end-1,4).*(X(end-1,5)+X(end-1,4)) ...
    + (X(end-1,4)+X(end-1,5)+X(end-1,6))^2; % objective function : square(linear velocity of hip) + square (angular velocity of trunk)

end