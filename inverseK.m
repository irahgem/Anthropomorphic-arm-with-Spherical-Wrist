function J = inverseK(Px, Py, Pz)
%input: end-effector position
%ouput: joint angles at each frames

%DH parameter
A = [90 0 90 -90 90 0] ;%twist angle
a = [0 1 0 0 0 0];      %offset as to xn
d = [1 0 0.25 1 0 0];   %offset as to z(n-1)

%DH model
T0_6 = [1 0 0 Px; 0 1 0 Py; 0 0 1 Pz; 0 0 0 1];

% Joint 5 position
P = [Px; Py; Pz];

%Determining Joint angles for frames 1,2,and 3
C1 = sqrt(P(1)^2+P(2)^2);
C2 = P(3)-d(1);
C3 = sqrt(C1^2+C2^2);
C4 = sqrt(a(3)^2+d(4)^2);
D1 = d(2)/C1;
D2 = (C3^2+a(2)^2-C4^2)/(2*a(2)*C3);
D3 = (a(2)^2+C4^2-C3^2)/(2*a(2)*C4);

a1 = atan2d(D1,sqrt(abs(1-D1^2)));
a2 = atan2d(sqrt(abs(1-D2^2)),D2);
b = atan2d(sqrt(abs(1-D3^2)),D3);
p1 = atan2d(P(2),P(1));
p2 = atan2d(C2,C1);

%Joint angles
J = [p1-a1 round(a2-p2) round(b-90)];

T = [];
for n = 1:3
       t = [cosd(J(n)) -sind(J(n))*cosd(A(n)) sind(J(n))*sind(A(n)) a(n)*cosd(J(n));
            sind(J(n)) cosd(J(n))*cosd(A(n)) -cosd(J(n))*sind(A(n)) a(n)*sind(J(n));
            0 sind(A(n)) cosd(A(n)) d(n);
            0 0 0 1];
        T = [T; {t}];
end
T0_3 = T{1}*T{2}*T{3};
T3_6 = inv(T0_3)*T0_6;

J4 = round(atan2d(T3_6(2,3),T3_6(1,3)));
J5 = round(atan2d(sqrt(abs(1-T3_6(3,3)^2)),T3_6(3,3)));
J6 = atan2d(T3_6(3,2),-T3_6(3,1));

J = [J J4 J5 J6];
if J(1,1) >= -180 && J(1,1) <= 180 && J(1,2)>= -180 ...
        && J(1,2) <= 180 && J(1,3) >= -180 && J(1,3) <= 180 ...
        && J(1,4) >= -180 && J(1,4) <= 180 && J(1,5) >= -180 ...
        && J(1,5) <= 180 && J(1,6) >= -180 && J(1,6) <= 180
else
    error('Resulting angles are out of range');
end
