%1
A = [-0.4 0 -0.01;1 0 0;-1.4 9.8 -0.02];
B = [6.3; 0; 9.3]; 
C = [0 0 1]%eye(3);
sys = ss(A,B,C,0);

sys1 = ss2tf(A,B,C,D)
step(sys)%, hold on, step(sys_d1,Tf)
impulse(sys)


%2
A = [-0.0895 -0.286 0;-0.0439 -0.272 0;0 1 0];
B = [0.0145; -0.0122; 0]; 
C = [0 0 1]%eye(3);
sys = ss(A,B,C,0);

[n,d]=ss2tf(A,B,C,D)

mySys_tf=tf(n,d)
step(sys)%, hold on, step(sys_d1,Tf)
impulse(sys)
