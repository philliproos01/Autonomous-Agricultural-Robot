A = [2 1 0; 1 0 0; 3 0 1];
B = [1; 0; 0]; 
C = [0 0 1];
D = 0;
xinit = [1 ; 0; 0]

sysDisc = ss(A, B, C, D);
%poles = [0, 0, 0]
%poles = [0.5, 0.5 + 0.01i, 0.5 + 0.01i]
poles = [0, 0.9, -0.9];

k = acker(A, B, poles);
%change the stuff below here
A1 = sysDisc.A - (sysDisc.B*k);
B1 = sysDisc.B;
C1 = sysDisc.C;
D1 = sysDisc.D;
sysn = ss(A1, B1, C1, D1);
[wn,zeta,poles] = damp(sysn);
n = 0:0.1:10;
x = initial(sysn, xinit, n);

stem(n,x,'LineWidth',2)
%figure()
