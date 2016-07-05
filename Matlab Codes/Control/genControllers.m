%%%%%%%%%%%%%%%%%%%%%%
%	Description: This function generates 2x2 MIMO PID controlers from the trajectories
%	of the optimized fall, as well as other parameters in opt_out_filename
%
%   Input:     opt_out_filename - name of csv file containing params from
%                   optimization
%              pi_in_filename - name of csv file to write output to  
%   
%   opt_out_filename stucture: 
%   
%   C1: t     - time
%   C2: Te_2  - torque applied at 2nd joint (at output shaft)
%   C3: Te_3  - torque applied at 3rd joint (at output shaft)
%   C4: a11   - moment of inertia in terms of state vars at time t (at
%   output shaft)
%   C5: a12
%   C6: a13
%   C7: a21
%   C8: a22
%   C9: a23
%   C10: a31
%   C11: a32
%   C12: a33
%   C13: c11  - net torques at each joint (at output shaft)
%   C14: c21
%   C15: c31
%
%   pi_in_filename structure:
%
%   C1: t               - time
%   C2: Te_2            - torque applied at 2nd joint (at armature)
%   C3: Te_3            - torque applied at 3rd joint (at armature)
%   C4: Te_2_response   - response of Te_2 due to load torques (at armature)
%   C5: Te_3_response   - response of Te_3 due to load torques (at armature)
%   C6: c1              -control matrix
%   C7: c2
%   C8: c3
%   C9: c4
%
%	Author Name: Todd Darcie
%   Student ID: 35296128
%   ENPH 459 Exsoskeleton safe-fall project
%
%%%%%%%%%%%%%%%%%%%%%%

opt_out_filename = 'stateVariables.csv';
pi_in_filename = 'control.csv';

global alpha beta J_rotor N
%motor params

syms s;

kt = 13.4*10^-3; %torque constant
kv = 1.4*10^-3*2*pi/60; %speed constant
R = 1.9; %armature resistance
N = 64; %speed reduction factor
J_rotor = 5.7e-7; %rotor inertia
alpha = 1/2; %length scaling factor
beta = 1/32; %mass scaling factor

%import variables from csv
M = csvread(opt_out_filename);
%scale results to alpha and beta scaling if optimization was at full scale 
%convert to rotor side of geartrain. Also Inverts A.
M = scaledata(M,1,1,1);

t = M(:,1);
dt_source = t(2)-t(1);
f_ssource = 1/dt_source;
Te_2 = M(:,2);
Te_3 = M(:,3);
a11 = M(:,4);
a12 = M(:,5);
a13 = M(:,6);
a21 = M(:,7);
a22 = M(:,8);
a23 = M(:,9);
a31 = M(:,10);
a32 = M(:,11);
a33 = M(:,12);
Tnet_1 = M(:,13);
Tnet_2 = M(:,14);
Tnet_3 = M(:,15);

%interpolate optimization data
fs = 202; %sampling frequency (Hz)
dt = 1/fs;
t_fine = 0:dt:t(length(t));
Te_2 = spline(t,Te_2,t_fine);
Te_3 = spline(t,Te_3,t_fine);
Tnet_1 = spline(t,Tnet_1,t_fine);
Tnet_2 = spline(t,Tnet_2,t_fine);
Tnet_3 = spline(t,Tnet_3,t_fine);

%derived mechanical load torques (includes friction)
Tm_1 = Tnet_1;
Tm_2 = Tnet_2 - Te_2;
Tm_3 = Tnet_3 - Te_3;

%find fft of input signal

wc_2 = findMaxOmega(Te_2,dt,1);
wc_3 = findMaxOmega(Te_3,dt,1);

%state space rep
K = kt*kv/R;

C = [-K -K -K 0 0 0; 0 0 0 -K -K -K];
D = [kt/R 0 0 0 0; 0 kt/R 0 0 0];

states = {'T21' 'T22' 'T23' 'T31' 'T32' 'T33'};
inputs = {'v1' 'v2' 'Tm1' 'Tm2' 'Tm3'};
outputs = {'Te2', 'Te3'};
L1 = length(a11);
T_2x5 = cell(L1,1);
T_2x2 = cell(L1,1);
T_2x3 = cell(L1,1);
C_2x2 = zeros(2,2,L1);

h = waitbar(0,'Generating MIMO controllers...');
for k = 1:L1
    
    waitbar(k / L1)
    
    A = [0 0 0 0 0 0;-K*a22(k) -K*a22(k) -K*a22(k) 0 0 0;0 0 0 -K*a23(k) -K*a23(k) -K*a23(k);0 0 0 0 0 0;-K*a32(k) -K*a32(k) -K*a32(k) 0 0 0;0 0 0 -K*a33(k) -K*a33(k) -K*a33(k)];
    
    B = [0 0 a21(k) 0 0;kt*a22(k)/R 0 0 a22(k) 0;0 kt*a23(k)/R 0 0 a23(k);0 0 a31(k) 0 0;...
        kt*a32(k)/R 0 0 a32(k) 0;0 kt*a33(k)/R 0 0 a33(k)];
    
    %transfer functions
    sys_mimo = ss(A,B,C,D,'statename',states,...
        'inputname',inputs,...
        'outputname',outputs);
    TF = tf(sys_mimo); %convert to tf object
    
    [num,den] = tfdata(TF);
    %subplot(2,5,1)
    for i = 1:2
       for j = 1:5
          %subplot(2,5,(i-1)*5+(j));
          n = num{i,j};
          d = den{i,j};
          tf_simple(i,j) = tf(n(1:3),d(1:3));
          %bodeplot(TF(i,j),tf_simple(i,j),'r--');
       end
    end
    T_2x5{k} = tf_simple;
    T_2x2{k} = tf_simple(:,1:2);
    T_2x3{k} = tf_simple(:,3:5);

    %generate controller
    C_2x2(:,:,k) = tf_to_C(T_2x2{k},wc_2,wc_3);

    %convert symbolic TF to tf object
end

close(h) 

%calculate system response from mechanical load torques
N = 10;
L2 = length(Te_2);
t = -N*dt:dt:N*dt;
Te_2_response = zeros(size(Te_2));
Te_3_response = Te_2_response;
Te_2_adj = Te_2_response;
Te_3_adj = Te_2_response;
v_2_expected = Te_2_response;
v_3_expected = Te_2_response;

Te_2_expected = Te_2_response;
Te_3_expected = Te_2_response;

h = waitbar(0,'Generating system response...');

for k = N+1:L2-(N+1)
    waitbar(k / L2)
    
    %T_2x3_tfd = c2d(T_2x3_tf,0.01);
    
    u = [Tm_1(k-N:k+N)' Tm_2(k-N:k+N)' Tm_3(k-N:k+N)'];
    y = lsim(T_2x3{floor(k*L1/L2)+1},u,t); %use transfer function closest to this time
    Te_2_response(k) = y(N+1,1);
    Te_3_response(k) = y(N+1,2);
    Te_2_adj(k) = Te_2(k)-Te_2_response(k);
    Te_3_adj(k) = Te_3(k)-Te_3_response(k);
end

close(h) 
h = waitbar(0,'Generating inverse system response...');

for k = N+1:L2-(N+1)
    waitbar(k / L2)
    u = [Te_2_adj(k-N:k+N)' Te_3_adj(k-N:k+N)'];
    y = lsim(inv(T_2x2{floor(k*L1/L2)+1}),u,t);
    v_2_expected(k) = y(N+1,1);
    v_3_expected(k) = y(N+1,2);
end
close(h) 

Te_2_response2 = lsim(T_2x3{20},[Tm_1' Tm_2' Tm_3'],t_fine);
v_2_expected2 = lsim(inv(T_2x2{25}),[Te_2_adj' Te_3_adj'],t_fine);

% h = waitbar(0,'test');
% 
% for k = N+1:L2-(N+1)
%     waitbar(k / L2)
%     u = [v_2_expected(k-N:k+N)' v_3_expected(k-N:k+N)'];
%     y = lsim(T_2x2{floor(k*L1/L2)+1},u,t);
%     Te_2_expected(k) = y(N+1,1);
%     Te_3_expected(k) = y(N+1,2);
% end
% close(h) 

Te_expected_0 = lsim(T_2x2{1},[v_2_expected' v_3_expected'],t_fine);
Te_expected_20 = lsim(T_2x2{20},[v_2_expected' v_3_expected'],t_fine);
Te_expected_40 = lsim(T_2x2{40},[v_2_expected' v_3_expected'],t_fine);

figure
subplot(2,1,1);
plot(t_fine,Te_2_expected,t_fine,Te_expected_0(:,1),t_fine,Te_expected_20(:,1),t_fine,Te_expected_40(:,1),t_fine,Te_2_adj);
legend('changing controllers','controler 1','controler 20','controler 40', 'adjusted desired torque signal');
subplot(2,1,2);
plot(t_fine,Te_3_expected,t_fine,Te_expected_0(:,2),t_fine,Te_expected_20(:,2),t_fine,Te_expected_40(:,2),t_fine,Te_3_adj);
legend('changing controllers','controler 1','controler 20','controler 40', 'adjusted desired torque signal');

%write data to csv
 C_2x2_p = permute(C_2x2,[3 2 1]);
 c1 = C_2x2_p(:,1,1);
 c2 = C_2x2_p(:,2,1);
 c3 = C_2x2_p(:,1,2);
 c4 = C_2x2_p(:,2,2);

M2 = [t_fine' Te_2' Te_3' Te_2_response' Te_3_response' c1 c2 c3 c4];
csvwrite(pi_in_filename,M2);

