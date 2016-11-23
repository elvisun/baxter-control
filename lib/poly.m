function output = poly( startpoint, endpoint)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Oo = convertRobotToModel(startpoint);
dOo = [0 0 0 0 0 0 0];
ddOo = [0 0 0 0 0 0 0];
Of = convertRobotToModel(endpoint);
dOf = [0 0 0 0 0 0 0];
ddOf = [0 0 0 0 0 0 0];

tf = 1;
samples = tf/10;
t = 0:samples:tf;

a0 = Oo
a1 = dOo
a2 = ddOo/2
a3 = (20*Of - 20*Oo-(8*dOf+12*dOo)*tf - (3*ddOo-ddOf)*tf^2)/(2*tf^3)
a4 = (30*Oo - 30*Of + (14*dOf+16*dOo)*tf + (3*ddOo-2*ddOf)*tf^2)/(2*tf^4)
a5 = (12*Of - 12*Oo-(6*dOf+6*dOo)*tf - (ddOo-ddOf)*tf^2)/(2*tf^5)

ret = [];
for i=1:length(t)
    O = a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3+a4*t(i)^4+a5*t(i)^5; 
    O = O + [150 60 60 50 140 63 135]
    ret = [ret; O];
    
end
Of = convertRobotToModel(endpoint)
output = ret;



