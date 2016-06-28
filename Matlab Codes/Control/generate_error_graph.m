global weight

weights = linspace(0,1,50);
weight = 0;

error1s = zeros(length(weights),1);
error2s = error1s;
error3s = error1s;

error1Ms = error1s;
error2Ms = error1s;
error3Ms = error1s;
error_tots = error1s;
error_totMs = error1s;
for i = 1:length(weights)
   weight = weights(i); 
   [space1, N1,error1, space2, N2,error2, space3, N3,error3,error_tot,error1M, error2M,error3M] = optimize_link_configuration_optimize(L1,L2,L3);
   error1s(i) = error1;
   error2s(i) = error2;
   error3s(i) = error3;
   
   error1Ms(i) = error1M;
   error2Ms(i) = error2M;
   error3Ms(i) = error3M;
   error_tots(i) = sqrt(error1^2+error2^2+error3^2);
   error_totMs(i) = sqrt(error1M^2+error2M^2+error3M^2);
end

figure(2)

subplot(2,1,1)
plot(weights,error1s,weights,error2s,weights,error3s, weights, error_tots)
legend('Link 1 relative R_g and COM error','Link 2 relative R_g and COM error','Link 3 relative R_g and COM error','Total relative R_g and COM error')
grid on
subplot(2,1,2)
plot(weights,error1Ms,weights,error2Ms,weights,error3Ms, weights, error_totMs)
legend('Link 1 relative mass error','Link 2 relative mass error','Link 3 relative mass error','Total relative mass error');
xlabel('Wieght');
grid on
