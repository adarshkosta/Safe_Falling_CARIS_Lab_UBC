%%% Plotting joint characteristcs
function Result_plotting(X_0,ini_guess_impact,Time)
global L0 L1 L2 var_array_length

% X(:,1) = Y(1:var_array_length);
% X(:,2) = Y(var_array_length+1:2*var_array_length);
% X(:,3) = Y(2*var_array_length+1:3*var_array_length);
% X(:,4) = Y(3*var_array_length+1:4*var_array_length);
% X(:,5) = Y(4*var_array_length+1:5*var_array_length);
% X(:,6) = Y(5*var_array_length+1:6*var_array_length);
% %% Computing the impact linear velocity at hip
% VV = zeros(optimal_result_impact-1,1); % Initializing the array corresponding to the vertical impact velocity of the optimal solution
% VH = zeros(optimal_result_impact-1,1); % Initializing the array corresponding to the horizontal impact velocity of the optimal solution
% VT = zeros(optimal_result_impact-1,1); % Initializing the array corresponding to the total impact velocity of the optimal solution
% 
% VV_0 = zeros(ini_guess_impact,1); % Initializing the array corresponding to the vertical impact velocity of the initial guess
% VH_0 = zeros(ini_guess_impact,1); % Initializing the array corresponding to the horizontal impact velocity of the initial guess
% VT_0 = zeros(ini_guess_impact,1); % Initializing the array corresponding to the total impact velocity of the initial guess
% 
% for j = 1:optimal_result_impact-1 % Calculating horizontal, vertical and total impact linear velocity of the optimal solution
%     VH(j) = -(L2.*sin(X(j,1)+X(j,2))+(L0+L1).*sin(X(j,1))).*X(j,4)-L2.*sin(X(j,1)+X(j,2)).*X(j,5); % Impact horizontal velocity
%     VV(j) = (L2.*cos(X(j,1)+X(j,2))+(L0+L1).*cos(X(j,1))).*X(j,4)+L2.*cos(X(j,1)+X(j,2)).*X(j,5); % Impact vertical velocity
%     VT(j) = sqrt(VV(j)^2+VH(j)^2); % Impact total velocity
% end

for j = 1:ini_guess_impact % Calculating horizontal, vertical and total linear velocity of the initial guess
    VH_0(j) = -(L2.*sin(X_0(j,1)+X_0(j,2))+(L0+L1).*sin(X_0(j,1))).*X_0(j,4)-L2.*sin(X_0(j,1)+X_0(j,2)).*X_0(j,5); % Impact horizontal velocity - initial guess
    VV_0(j) = (L2.*cos(X_0(j,1)+X_0(j,2))+(L0+L1).*cos(X_0(j,1))).*X_0(j,4)+L2.*cos(X_0(j,1)+X_0(j,2)).*X_0(j,5); % Impact vertical velocity - initial guess
    VT_0(j) = sqrt(VV_0(j)^2+VH_0(j)^2); % Impact total velocity - initial guess
end
%% plotting and comparing the impact linear velocity for the cases of initial guess and optimal solution
figure
hold on
box on
% plot (Time(1:optimal_result_impact-1),VV,Time(1:optimal_result_impact-1),VH,'r',Time(1:optimal_result_impact-1),VT,'k');
plot (Time(1:ini_guess_impact),VV_0,'--b',Time(1:ini_guess_impact),VH_0,'--r',Time(1:ini_guess_impact),VT_0,'--k');
xlabel('Time(s)');
ylabel('Hip linear velocity (m/s)');
legend('Vertical velocity','Horizontal velocity','Total velocity','IG-Vertical velocity','IG-Horizontal velocity','IG-Total velocity','Location','southwest')
hold off
%% plotting and comparing the joint angle and angular velocity for the cases of initial guess and optimal solution
figure
hold on
plot(Time(1:optimal_result_impact-1),Y(1:optimal_result_impact-1),'g',...
    Time(1:optimal_result_impact-1),Y(var_array_length+1:var_array_length+optimal_result_impact-1),...
    Time(1:optimal_result_impact-1),Y(2*var_array_length+1:2*var_array_length+optimal_result_impact-1),'k');
plot(Time(1:ini_guess_impact),X_0(1:ini_guess_impact,1),'--g',Time(1:ini_guess_impact),X_0(1:ini_guess_impact,2),'--b',Time(1:ini_guess_impact),X_0(1:ini_guess_impact,3),'--k');
legend('TetaA','TetaK','TetaH','IG-TetaA','IG-TetaK','IG-TetaH','Location','southwest');
xlabel('Time(s)');
ylabel('Joint angle(rad)');

figure
hold on
plot(Time(1:optimal_result_impact-1),Y(3*var_array_length+1:3*var_array_length+optimal_result_impact-1),'g',...
    Time(1:optimal_result_impact-1),Y(4*var_array_length+1:4*var_array_length+optimal_result_impact-1),...
    Time(1:optimal_result_impact-1),Y(5*var_array_length+1:5*var_array_length+optimal_result_impact-1),'k');
plot(Time(1:ini_guess_impact),X_0(1:ini_guess_impact,4),'--g',Time(1:ini_guess_impact),X_0(1:ini_guess_impact,5),'--b',Time(1:ini_guess_impact),X_0(1:ini_guess_impact,6),'--k');
legend('dTetaA','dTetaK','dTetaH','IG-dTetaA','IG-dTetaK','IG-dTetaH','Location','southwest');
xlabel('Time(s)');
ylabel('Joint angular velocity(rad/s)');

%% plotting and comparing the applied torque at the joints for the cases of initial guess and optimal solution
figure
hold on
box on
plot(Time(1:optimal_result_impact-1),Y(6*var_array_length+1:6*var_array_length+optimal_result_impact-1),...
    'g',Time(1:optimal_result_impact-1),Y(7*var_array_length+1:7*var_array_length+optimal_result_impact-1),'k');
plot(Time(1:ini_guess_impact),X_0(1:ini_guess_impact,7),'--g',Time(1:ini_guess_impact),X_0(1:ini_guess_impact,8),'--k');
legend('Knee-torque','Hip-torque','IG-Knee-torque','IG-Hip-torque','Location','southwest');
xlabel('Time(s)');
ylabel('Joint Torque(N.m)');
hold off
end