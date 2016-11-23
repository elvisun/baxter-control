%% Init goals
%  Sets up goal trajectories for controller to use.

%% Discrete goals
%  Get starting position from Baxter's current position
states = ones(16,1)*-1000;
bax_sub = rossubscriber('/robot/joint_states', rostype.sensor_msgs_JointState);
%  Get updates for all joints
msg = receive(bax_sub);
states = joint_states(msg,length(msg.Position),states);
while (min(states) < -500)
    % Get another message
    msg = receive(bax_sub);
    % Get states from it
	states = joint_states(msg);
end
initial_position = [0 states'];

%% Goals
%  Setup your goals and interpolate positions here
goals = [initial_position];

%   format   ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
first = [5, -0.036432043712278574, -0.0023009711818281204, 0.7531845668517382, -0.0015339807878854137, -0.54686415088115, -0.0030679615757708274, 1.2547962844902685, 0.0019174759848567672, 0.004218447166684887, 0.7608544707911652, 0.014956312681882784, -0.5522330836387489, 0.0030679615757708274, 1.2482768661417554, 0.0011504855909140602, -12.565987119160338];
second = [10, -0.03681553890924993, 0.41302432713814763, 0.48895637613847565, 0.18676216092504913, -0.6596117387907279, -0.055223308363874894, 0.46479617872928036, 0.7282573790486002, -0.4345000581685434, 0.4736165682596215, -0.18906313210687725, -0.6711165946998685, 0.05560680356084625, 0.438335010138257, -0.768140879533621, -12.565987119160338];
third = [15, -0.03566505331833587, 0.4341165629715721, 1.7337817855074888, -0.024927187803137973, -0.0222427214243385, -0.23853401251618184, -0.02569417819708068, 2.345456624676798, -0.44677190447162674, 1.7813351899319367, 0.02914563496982286, -0.005368932757598948, 0.24735440204652295, -0.09127185687918211, -2.450150813449977, -12.565987119160338];
fourth = [20, -0.03604854851530722, 0.5779272618358297, 0.14227671807637213, -0.7727428218972772, -0.27381557063754636, -3.041116911982833, -0.8736020587007431, 2.6403644311477685, -0.6024709544419963, 0.10776215034895031, 0.8275826350641807, -0.27650003701634585, 3.0549227390738016, -0.9533690596707847, -2.58974306514755, -12.565987119160338];
points = [first;second;third;fourth];
output = [];
for index = 1:3
    Oo = points(index,:);
    dOo = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    ddOo = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    Of = points((index+1),:);
    dOf = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    ddOf = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    
    tf = 1;
    samples = tf/10;
    t = 0:samples:tf;
    
    a0 = Oo;
    a1 = dOo;
    a2 = ddOo/2;
    a3 = (20*Of - 20*Oo-(8*dOf+12*dOo)*tf - (3*ddOo-ddOf)*tf^2)/(2*tf^3);
    a4 = (30*Oo - 30*Of + (14*dOf+16*dOo)*tf + (3*ddOo-2*ddOf)*tf^2)/(2*tf^4);
    a5 = (12*Of - 12*Oo-(6*dOf+6*dOo)*tf - (ddOo-ddOf)*tf^2)/(2*tf^5);
    
    ret = [];
    for i=1:length(t)
        O = a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3+a4*t(i)^4+a5*t(i)^5;
        ret = [ret; O];
    end
     goals = [goals; ret];
end