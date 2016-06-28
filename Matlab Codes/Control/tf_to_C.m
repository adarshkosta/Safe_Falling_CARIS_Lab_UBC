function C = tf_to_C(T_2x2,wc_2,wc_3)
sys1 = T_2x2(1,1);
sys2 = T_2x2(1,2);
sys3 = T_2x2(2,1);
sys4 = T_2x2(2,2);

opts = pidtuneOptions('PhaseMargin',70);
C1 = pidtune(sys1,'I',wc_2,opts);
C2 = pidtune(sys2,'I',wc_3,opts);
C3 = pidtune(sys3,'I',wc_2,opts);
C4 = pidtune(sys4,'I',wc_3,opts);

C = [C1.Ki 0;0 C4.Ki];
end