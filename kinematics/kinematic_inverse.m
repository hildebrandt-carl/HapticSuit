clear
clc

% The answer we want
Fx = 0 ;
Fy = 0 ; 
Fz = 4 ;
Tx = -2 ;
Ty = -2 ;
Tz = -6 ;

syms w1 w2 w3 w4 q1 q2 q3 q4

assume(w1,'positive')
assume(w2,'positive')
assume(w3,'positive')
assume(w4,'positive')
assume(q1>0 & q1<2*pi)
assume(q2>0 & q2<2*pi)
assume(q3>0 & q3<2*pi)
assume(q4>0 & q4<2*pi)

ky = 2 ;
km = 3 ;

% Run the kinematic equations
run("kinematic_equations.m")

eq_Fx = fx1 + fx2 + fx3 + fx4 - Fx ;
eq_Fy = fy1 + fy2 + fy3 + fy4 - Fy ;
eq_Fz = fz1 + fz2 + fz3 + fz4 - Fz ;

eq_Tx = (tx1 + tx2 + tx3 + tx4) + ((fz1 - fz2) + (fy1 - fy2)) - Tx ;
eq_Ty = (ty1 + ty2 + ty3 + ty4) + ((fz3 - fz4) + (fx3 - fx4)) - Ty ;
eq_Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4)) - Tz ;


Equations = [eq_Fx,eq_Fy,eq_Fz,eq_Tx,eq_Ty,eq_Tz] ;
Variables = [w1, w2, w3, w4, q1, q2, q3, q4] ; 
[w1, w2, w3, w4, q1, q2, q3, q4] = vpasolve(Equations, Variables) ;


fprintf('w1 %f      w2 %f      w3 %f       w4 %f\n',w1, w2, w3, w4);
fprintf('q1 %f      q2 %f      q3 %f       q4 %f\n',q1, q2, q3, q4);
