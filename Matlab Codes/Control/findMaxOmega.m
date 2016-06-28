function omega_max = findMaxOmega(Te_2_adj,dt,plot_flag)

 L = length(Te_2_adj); %number of points 
 Fs = 1/dt; % Sampling frequency

 
 NFFT = 2^(nextpow2(L)-1);
 
 x=zeros(NFFT,1); x(1:NFFT,1) = Te_2_adj(1,1:NFFT)';
 
 YY = fft(x,NFFT)/L; 
 ff = Fs/2*linspace(0,1,NFFT/2+1);
 
 Y = 2*abs(YY(1:NFFT/2+1));
 omega = 2*pi*ff;
 if(plot_flag == 1)
     
     figure(1)
     clf
    hold on
    % Plot single-sided amplitude spectrum. 
    plot(omega,Y);
    title('Single-Sided Amplitude Spectrum of Te_2')
    xlabel('Frequency (rad/s)')
    ylabel('|Y|')
    
 end

 
 full = trapz(omega,Y);
 part = 0; 
 N_p = 2;
 thresh = 0.8;
 while part < thresh*full
    part = trapz(omega(1:N_p),Y(1:N_p));
    N_p = N_p + 1;
 end
 
 omega_max = omega(N_p);

 
  if(plot_flag == 1)
    y1=get(gca,'ylim');
    plot([omega_max omega_max],y1,'r--')
    grid
    hold off
 end
end