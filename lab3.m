function lab3()
symbolic = 1;
Tentries = [3,4];
firstset = 0;
secondset = 0;

%Symbolic
if symbolic
    syms theta1 theta2 theta3 theta4 theta5 theta6;
end

%First Set of Angles
if firstset
    theta1 = deg2rad(12.1); theta2 = deg2rad(-49.26); theta3 = deg2rad(46.06);
    theta4 = deg2rad(-88.43); theta5 = deg2rad(-91.15); theta6 = deg2rad(-0.72);
end

%Second Set of Angles
if secondset
    theta1 = deg2rad(-15.4); theta2 = deg2rad(-49.28); theta3 = deg2rad(80.62);
    theta4 = deg2rad(-121.23); theta5 = deg2rad(-92.67); theta6 = deg2rad(-2.77);
end

%Calculate DH Matrices
    A1 = calcA(0, -pi/2, 152, theta1);
    A2 = calcA(244, 0, 120, theta2);
    A3 = calcA(213, 0, -93, theta3);
    A4 = calcA(0, -pi/2, 83, theta4);
    A5 = calcA(0, pi/2, 83, theta5);
    A6 = calcA(58, pi, 134, theta6);
    T = A1*A2*A3*A4*A5*A6;
    
%Symbolic
if symbolic
    [A, B] = coeffs(T(Tentries(1), Tentries(2)));
    A(A<0.001) = 0;
    Tnew = dot(A,B);
    digits(4);
    fprintf('T matrix entry %d, %d\n', Tentries(1), Tentries(2));
    pretty(vpa(simplify(Tnew)));
end

%First Set of Angles - Error
if firstset
    r1 = [440; 250; 215];
    error1 = sqrt(sum((T(1:3,4) - r1).^2));
    fprintf('Error1 is %.4f meters\n', error1/1000);
end
%Second Set of Angles - Error
if secondset
    r2 = [456; 47; 93];
    error2 = sqrt(sum((T(1:3,4) - r2).^2));
    fprintf('Error2 is %.4f meters\n', error2/1000);
end
end

function A = calcA(a, alpha, d, theta)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
        0 sin(alpha) cos(alpha) d; 0 0 0 1];
end

function rad = deg2rad(angle)
    rad = angle*pi/180;
end