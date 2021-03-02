% 推导尾推公式
clear all
clc

syms mux muy muz
syms p rb
syms a1c a1s lam0 lam1s lam1c
syms th0 tht
syms C %1/2*s*a0/b
syms k %delta/a0

UT = mux*sin(p)+muy*cos(p)+rb;

UP = rb*(a1c*cos(p)+a1s*sin(p))+ muz - lam0;
    
th = th0+rb*tht;

%%
C_Zpbl = -C*int( sqrt(UP^2+UT^2)*(UP*(th+atan(UP/UT))+k*UT) , 'rb',0,1);

C_Zpbl0 = simplify(int(C_Zpbl/(-C), 'p', -pi, pi)/(2*pi));

C_Zpbl1c = simplify(int(C_Zpbl/(-C)*cos(p) , 'p', -pi, pi)/(pi));

C_Zpbl1s = simplify(int(C_Zpbl/(-C)*sin(p), 'p', -pi, pi)/(pi));

%%
C_ypbl = C*int( sqrt(UP^2+UT^2)*(-UT*(th+atan(UP/UT))+k*UP) , 'rb',0,1);