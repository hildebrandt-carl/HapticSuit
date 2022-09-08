clear
clc

% The answer we want
Fx = 0 ;
Fy = 0 ; 
Fz = 8 ;
Tx = 0 ;
Ty = 0 ;
Tz = 0 ;

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

eq_Fx = Fx == fx1 + fx2 + fx3 + fx4 ;
eq_Fy = Fy == fy1 + fy2 + fy3 + fy4 ;
eq_Fz = Fz == fz1 + fz2 + fz3 + fz4 ;

eq_Tx = Tx == (tx1 + tx2 + tx3 + tx4) + ((fz1 - fz2) + (fy1 - fy2)) ;
eq_Ty = Ty == (ty1 + ty2 + ty3 + ty4) + ((fz3 - fz4) + (fx3 - fx4)) ;
eq_Tz = Tz == (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4)) ;


EqSys = [eq_Fx,eq_Fy,eq_Fz,eq_Tx,eq_Ty,eq_Tz] ;
S = solve(EqSys) ;

disp("w1: ")
S.w1

disp("w2: ")
S.w2

disp("w3: ")
S.w3

disp("w4: ")
S.w4

disp("q1: ")
S.q1

disp("q2: ")
S.q2

disp("q3: ")
S.q3

disp("q4: ")
S.q4






