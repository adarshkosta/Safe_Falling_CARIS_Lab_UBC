%%%%%%%%%%%%%%%%%%%%%%
%	Description: This function converts torque trajectories and inertias
%	from the full size to scaled down model and from the output-side to
%	rotor-side of the motors. 
%
%   Input:     M -  matrix containing interias, torques and times
%              scale_size - boolean to convert from full-size to scaled down model
%              scale_gearbox - boolean to convert to rotor-side of motors
%   
%	Author Name: Todd Darcie
%   Student ID: 35296128
%   ENPH 459 Exsoskeleton safe-fall project
%
%%%%%%%%%%%%%%%%%%%%%%


function M = scaledata(M,scale_size,scale_gearbox,add_J_rotor)

global alpha beta N J_rotor

t = M(:,1);
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

if scale_size == 1
   t =  t*sqrt(alpha);
   Te_2 = Te_2*(alpha*beta);
   Te_3 = Te_3*(alpha*beta);
   Tnet_1 = Tnet_1*(alpha*beta);
   Tnet_2 = Tnet_2*(alpha*beta);
   Tnet_3 = Tnet_3*(alpha*beta);
   a11 = a11*(alpha^2*beta);
   a12 = a12*(alpha^2*beta);
   a13 = a13*(alpha^2*beta);
   a21 = a21*(alpha^2*beta);
   a22 = a22*(alpha^2*beta);
   a23 = a23*(alpha^2*beta);
   a31 = a31*(alpha^2*beta);
   a32 = a32*(alpha^2*beta);
   a33 = a33*(alpha^2*beta);
end

if scale_gearbox == 1
   Te_2 = Te_2/N;
   Te_3 = Te_3/N;
   Tnet_1 = Tnet_1/N;
   Tnet_2 = Tnet_2/N;
   Tnet_3 = Tnet_3/N;
   a11 = a11/N^2;
   a12 = a12/N^2;
   a13 = a13/N^2;
   a21 = a21/N^2;
   a22 = a22/N^2;
   a23 = a23/N^2;
   a31 = a31/N^2;
   a32 = a32/N^2;
   a33 = a33/N^2;
end

if add_J_rotor == 1
     a22 = a22+J_rotor;
     a33 = a33+J_rotor;
end
% 
% for i = 1:length(a11)
%     A = [a11(i) a12(i) a13(i);a21(i) a22(i) a23(i);a31(i) a32(i) a33(i)];
%     A = inv(A);
%     a11(i) = A(1,1);
%     a12(i) = A(1,2);
%     a13(i) = A(1,3);
%     a21(i) = A(2,1);
%     a22(i) = A(2,2);
%     a23(i) = A(2,3);
%     a31(i) = A(3,1);
%     a32(i) = A(3,2);
%     a33(i) = A(3,3);
% end

M = [t Te_2 Te_3 1./a11 1./a12 1./a13 1./a21 1./a22 1./a23 1./a31 1./a32 1./a33...
    Tnet_1 Tnet_2 Tnet_3];
end