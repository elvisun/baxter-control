n = 3;
syms l2 l6 t1 qd qdd dt1 dl2 dl6 ddt1 ddl2 ddl6 tau1 tau2 tau3 tau;
q = [t1; l2; l6];
qd = [dt1; dl2; dl6];
qdd = [ddt1; ddl2; ddl6];

% link lengths in meters
l1 = 0.2;         
l3 = 0.05;
l4 = 0.05;
l5 = 0.05;

% center of mass for each link
lc1 = 0.1;
lc2 = l2/2;
lc3 = l3+l6/2;

%         a     d       alpha   theta
matrix = [0     l1      0       t1;         % the parameters found using 
          l4    l2+l5   pi/2    pi;          % the DH convention
          0     l6+l3   0       0;];

A = eye(4);
z0 = A(1:3,3);
o0 = A(1:3,4);
for index = 1:n
    A = A * [
        cos(matrix(index,4))    -sin(matrix(index,4))*cos(matrix(index,3))  sin(matrix(index,4))*sin(matrix(index,3))   matrix(index,1)*cos(matrix(index,4));
        sin(matrix(index,4))    cos(matrix(index,4))*cos(matrix(index,3))   -cos(matrix(index,4))*sin(matrix(index,3))  matrix(index,1)*sin(matrix(index,4));
        0                       sin(matrix(index,3))                        cos(matrix(index,3))                        matrix(index,2);
        0                       0                                           0                                           1
    ];
    if index == 1
        A01 = A;
        z1 = A(1:3,3);
        o1 = [0; 0; lc1];
        j1 = [cross(z0,(o1-o0)) zeros(3,2); z0 zeros(3,2)];
        jvc1 = j1(1:3,:);
        jwc1 = j1(4:6,:);
    end
    if index == 2
        A02 = A;
        z2 = A(1:3,3);
        o2 = [-l4*cos(t1); -l4*sin(t1); l1+lc2];
        j2 = [cross(z0,(o2-o0)) z1 zeros(3,1); z0 zeros(3,2)];
        jvc2 = j2(1:3,:);
        jwc2 = j2(4:6,:);
    end
    if index == 3
        A03 = A;
        z3 = A(1:3,3);
        o3 = [  -sin(t1)*(lc3)-l4*cos(t1); 
                cos(t1)*(lc3)-l4*sin(t1);
                l1 + l2 + l5];
        j3 = [cross(z0,(o3-o0)) z1 z2; z0 zeros(3,2)];
        jvc3 = j3(1:3,:);
        jwc3 = j3(4:6,:);
    end
end

% link masses in kg
m1 = 0.5;
m2 = 0.5;
m3 = 0.5;
% rotational matricies
r1 = A01(1:3,1:3);
r2 = A02(1:3,1:3);
r3 = A03(1:3,1:3);
% link inertias in Kgm^2
I1 = 0.01;
I2 = 0.01;
I3 = 0.01;

% inertia matrices
k1 = m1*(jvc1).'*jvc1+(jwc1).'*r1*I1*(r1).'*jwc1;
k2 = m2*(jvc2).'*jvc2+(jwc2).'*r2*I2*(r2).'*jwc2;
k3 = m3*(jvc3).'*jvc3+(jwc3).'*r3*I3*(r3).'*jwc3;
D = k1+k2+k3;

% kinetic energy
K = (1/2)*(qd).'*D*qd;

% Christoffel Symbols
syms c
for i = 1:n 
    for j = 1:n 
        for k = 1:n 
            c(i,j,k) = (1/2)*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)));
        end
    end
end

% potential energy 
g = 9.81; % gravity in m/s^2
p1 = m1*g*o1(3,1);
p2 = m2*g*o2(3,1);
p3 = m3*g*o3(3,1);
P = p1+p2+p3;

% gravity vector
g1 = diff(P,t1);
g2 = diff(P,l2);
g3 = diff(P,l6);
g = [g1; g2; g3];

% equations of motion
tau = [tau1;tau2;tau3];
Cpart = 0;
Dpart = 0;
Ctotal = 0;
for k = 1:n
    for j = 1:n
        for i = 1:n
            Cpart = Cpart + c(i,j,k)*qd(i);
        end
        Dpart = Dpart + D(k,j)*qdd(j);
        Ctotal = Ctotal + Cpart*qd(j);
    end
    tau(k) = Dpart + Ctotal + g(k);
end
simplify(tau(k))
