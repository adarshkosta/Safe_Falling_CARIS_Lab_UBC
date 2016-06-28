%NOT part of MAIN Code
%Matrix Inverse Alternative Testing

a11 = 1;
a12 = 4;
a13 = 1;
a21 = 5;
a22 = 3;
a23 = 2;
a31 = 4;
a32 = 1;
a33 = 6;

c11 = 1;
c21 = 2;
c31 = 5;

A =([a11 a12 a13 ; a21 a22 a23 ; a31 a32 a33]);
B = inv(A);
C = [c11 ; c21 ; c31];
D = ([a11 a12 a13 ; a21 a22 a23 ; a31 a32 a33])\[c11 ; c21 ; c31];