function tau = assignment5(pos,vel,acc)

n = 3;
q = pos;
qd = vel;
qdd = acc;

%   joint variables
t1 = q(1,1);    % theta 1 is the angle of the first joint 
l2 = q(2,1);    % 12 is the length of the first prismatic joint 
l6 = q(3,1);    % 16 is the length of the last joint

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
        % calculating the jacobian for link 1
        fprintf('\n\nThe Jacobian for link 1 is given by: \n')
        A01 = A;
        z1 = A(1:3,3);
        o1 = [0; 0; lc1];
        j1 = [cross(z0,(o1-o0)) zeros(3,2); z0 zeros(3,2)]
        jvc1 = j1(1:3,:);
        jwc1 = j1(4:6,:);
    end
    if index == 2
        % calculating the jacobian for link 2
        fprintf('\nThe Jacobian for link 2 is given by: \n')
        A02 = A;
        z2 = A(1:3,3);
        o2 = [-l4*cos(t1); -l4*sin(t1); l1+lc2];
        j2 = [cross(z0,(o2-o0)) z1 zeros(3,1); z0 zeros(3,2)]
        jvc2 = j2(1:3,:);
        jwc2 = j2(4:6,:);
    end
    if index == 3
        % calculating the jacobian for link 3
        fprintf('\nThe Jacobian for link 3 is given by: \n')
        A03 = A;
        z3 = A(1:3,3);
        o3 = [  -sin(t1)*(lc3)-l4*cos(t1); 
                cos(t1)*(lc3)-l4*sin(t1);
                l1 + l2 + l5];
        j3 = [cross(z0,(o3-o0)) z1 z2; z0 zeros(3,2)]
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
fprintf('\n\n The inertia matrix for this arm is: \n')
D = k1+k2+k3

% kinetic energy
fprintf('\n\n The kinetic energy (Joules) for this configuration is: \n')
K = (1/2)*(qdd)'*D*qdd

% Christoffel Symbols (the rest are zero, shown by script symversion)

c(1,3,1) = (sin(t1)*(cos(t1)/20 + sin(t1)*(l6/2 + 1/20)))/8 - (cos(t1)*(sin(t1)/40 - (cos(t1)*(l6/2 + 1/20))/2))/4 - (cos(t1)*(sin(t1)/20 - cos(t1)*(l6/2 + 1/20)))/8 + (sin(t1)*(cos(t1)/40 + (sin(t1)*(l6/2 + 1/20))/2))/4;
c(3,1,1) = (sin(t1)*(cos(t1)/20 + sin(t1)*(l6/2 + 1/20)))/8 - (cos(t1)*(sin(t1)/40 - (cos(t1)*(l6/2 + 1/20))/2))/4 - (cos(t1)*(sin(t1)/20 - cos(t1)*(l6/2 + 1/20)))/8 + (sin(t1)*(cos(t1)/40 + (sin(t1)*(l6/2 + 1/20))/2))/4;
c(1,1,3) = (cos(t1)*(sin(t1)/20 - cos(t1)*(l6/2 + 1/20)))/8 + (cos(t1)*(sin(t1)/40 - (cos(t1)*(l6/2 + 1/20))/2))/4 - (sin(t1)*(cos(t1)/20 + sin(t1)*(l6/2 + 1/20)))/8 - (sin(t1)*(cos(t1)/40 + (sin(t1)*(l6/2 + 1/20))/2))/4;
fprintf('\n The Christoffel symbols for this arm are: \n')
c

% potential energy 
g = 9.81; % gravity in m/s^2
p1 = m1*g*o1(3,1);
p2 = m2*g*o2(3,1);
p3 = m3*g*o3(3,1);
fprintf('\n The potential energy (Joules) for this configuration is: \n')
P = p1+p2+p3

% gravity vector (from symversion script)
g1 = 0;
g2 = 2943/400;
g3 = 0;
g = [g1; g2; g3];

% equations of motion
tau =[0 0 0]';
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
fprintf('\n The required force/torque (N or Nm) for each joint is: \n')
tau
end