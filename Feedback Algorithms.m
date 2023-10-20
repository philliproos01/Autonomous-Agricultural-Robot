%5
A = [-0.4 0 -0.01;1 0 0;-1.4 9.8 -0.02];
B = [6.3; 0; 9.3]; 
C = [0 0 1]%eye(3);
sys_c = ss(A,B,C,0);

step(sys_c)%, hold on, step(sys_d1,Tf)
impulse(sys)

%6
A = [-0.0895 -0.286 0;-0.0439 -0.272 0;0 1 0];
B = [0.0145; -0.0122; 0]; 
C = [0 0 1]%eye(3);
sys = ss(A,B,C,0);

step(sys)%, hold on, step(sys_d1,Tf)
impulse(sys)
