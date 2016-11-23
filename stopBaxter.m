%% StopBaxter
%  Stops Baxter by returning to position control

%% Get current position from robot
%  Get starting position from Baxter's current position
%  This might not work on the real Baxter if the wrong message type is
%  received, but that doesn't matter since it returns to position control
%  on its own
bax_sub = rossubscriber('/robot/joint_states', rostype.sensor_msgs_JointState);
%  Get updates for all joints
msg = receive(bax_sub);
states = joint_states(msg,length(msg.Position),states);
%  Put into separate containers
positionsL = states(1:7);
positionsR = states(8:14); 
torques = zeros(1,7);

%% Get publisher and message
pubL = rospublisher('robot/limb/left/joint_command');
pubR = rospublisher('robot/limb/right/joint_command');
msgL = rosmessage(pubL);
msgR = rosmessage(pubR);
msgL.Mode = int32(1); % Return to Baxter's internal position control
msgR.Mode = int32(1);
% msg.Mode = int32(3); % torque instead

%% Handle Command field
msgL.Command = positionsL; %the position you want it to be
msgR.Command = positionsR;
% msg.Command = torques; % torque instead

%% Handle Names field
% What each joint in the command message is - just controlling the left arm
namesL={'left_s0','left_s1','left_e0','left_e1','left_w0','left_w1',...
        'left_w2'};
msgL.Names = namesL;
namesR={'right_s0','right_s1','right_e0','right_e1','right_w0',...
        'right_w1','right_w2'};
msgR.Names = namesR;

%% Publish command
send(pubL,msgL);
send(pubR,msgR);
