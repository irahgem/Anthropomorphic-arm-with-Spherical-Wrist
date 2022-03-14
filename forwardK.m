function [result] = forwardK(theta1,theta2,theta3,theta4,theta5,theta6)

%DH parameters
J = [theta1 theta2 theta3 theta4 theta5 theta6]; %joint angles
A = [90 0 90 -90 90 0] ; %twist angle
a = [0 1 0 0 0 0];       %offset as to xn
d = [1 0 0.25 1 0 0];    %offset as to z(n-1)

%forward kinematics
if J(1,1) >= -180 && J(1,1) <= 180 && J(1,2)>= -180 ...
        && J(1,2) <= 180 && J(1,3) >= -180 && J(1,3) <= 180 ...
        && J(1,4) >= -180 && J(1,4) <= 180 && J(1,5) >= -180 ...
        && J(1,5) <= 180 && J(1,6) >= -180 && J(1,6) <= 180
    T = [];
    
    %Homogeneus Transformation
    for n = 1:6
        matT = [cosd(J(n)) -sind(J(n))*cosd(A(n)) sind(J(n))*sind(A(n)) a(n)*cosd(J(n));
                sind(J(n)) cosd(J(n))*cosd(A(n)) -cosd(J(n))*sind(A(n)) a(n)*sind(J(n));
                0 sind(A(n)) cosd(A(n)) d(n);
                0 0 0 1];
        T = [T; {matT}];
    end
    
    %Joint Positions
    P = [];
    for i = 1:6
        if i == 1
            P = [P,{T{i}}];
        else 
            matP = P{i-1}*T{i};
            P = [P, {matP}];
        end
    end
    
    result = [P{6}(1,4), P{6}(2,4), P{6}(3,4)];
else
    error('Not found');
end
end