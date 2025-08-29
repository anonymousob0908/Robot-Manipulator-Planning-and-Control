% fkine
function [T06] = myfkine(Angle_T,a,d,alpha)
syms thetas as ds al
theta1 = Angle_T(1, 1);
theta2 = Angle_T(1, 2);
theta3 = Angle_T(1, 3);
theta4 = Angle_T(1, 4);
theta5 = Angle_T(1, 5);
theta6 = Angle_T(1, 6);
d1 = d(1); d2 = d(2); d3 = d(3); d4 = d(4); d5 = d(5); d6 = d(6);
a1 = a(1); a2 = a(2); a3 = a(3); a4 = a(4); a5 = a(5); a6 = a(6); 
alpha1 = alpha(1); alpha2 = alpha(2); alpha3 = alpha(3); 
alpha4 = alpha(4); alpha5 = alpha(5); alpha6 = alpha(6); 


Ti_1i = [      cos(thetas)           -sin(thetas)           0              as;
       sin(thetas)*cos(al)    cos(thetas)*cos(al)    -sin(al)     -sin(al)*ds;
       sin(thetas)*sin(al)    cos(thetas)*sin(al)     cos(al)      cos(al)*ds;
                         0                     0            0               1];
% 需要修改偏置
T01 = subs(Ti_1i, [thetas al as ds],[theta1 alpha1 a1 d1]);
T12 = subs(Ti_1i, [thetas al as ds],[theta2-pi/2 alpha2 a2 d2]);
T23 = subs(Ti_1i, [thetas al as ds],[theta3 alpha3 a3 d3]);
T34 = subs(Ti_1i, [thetas al as ds],[theta4+pi/2 alpha4 a4 d4]);
T45 = double(subs(Ti_1i, [thetas al as ds],[theta5 alpha5 a5 d5]));
T56 = double(subs(Ti_1i, [thetas al as ds],[theta6 alpha6 a6 d6]));

T06 = double(T01*T12*T23*T34*T45*T56);
end
