clear
clc

w1 = 0 ;
w2 = 1 ;
w3 = 0 ;
w4 = 1 ;
q1 = deg2rad(0) ; 
q2 = deg2rad(0) ;
q3 = deg2rad(0) ;
q4 = deg2rad(0) ;

ky = 2 ;
km = 3 ;

% Run the kinematic equations
run("kinematic_equations.m")

Fx = fx1 + fx2 + fx3 + fx4 ;
Fy = fy1 + fy2 + fy3 + fy4 ;
Fz = fz1 + fz2 + fz3 + fz4 ;

Tx = (tx1 + tx2 + tx3 + tx4) + ((fz1 - fz2) + (fy1 - fy2)) ;
Ty = (ty1 + ty2 + ty3 + ty4) + ((fz3 - fz4) + (fx3 - fx4)) ;
Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4)) ;

fprintf('Fx %f      Fy %f      Fz %f\n',Fx, Fy, Fz);
fprintf('Tx %f      Ty %f      Tz %f\n',Tx, Ty, Tz);

