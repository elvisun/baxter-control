% Defining the Model Robot (link lengths may be different)
Link1 = Link('a',0,'d',170,'alpha',pi/2);
Link2 = Link('a',175,'d',0,'alpha',0);
Link3 = Link('a',0,'d',0,'alpha',pi/2);
Link4 = Link('a',0,'d',110,'alpha',-pi/2);
Link5 = Link('a',0,'d',0,'alpha',pi/2);
Link6 = Link('a',0,'d',250,'alpha',0);

ModelBot = SerialLink([Link1, Link2, Link3, Link4, Link5, Link6],'name','MSE');

% Joint Limits should be defined (in terms of actual robot angles in degrees)
Q_UpperLimits = [300, 270, 260, 300, 256, 300, 210];
Q_LowerLimits = [0, 30, 30, 0, 42, 0, 133];

% Sample list of via points (7 angles in degrees per point) to be sent to the robot
% Can be found by manually moving the robot along the path and recording key points
% "..." is used to brea k a statement over several lines in MATLAB
Q_via = [
    %ba sh1 elb rol pit yaw grip
    190 143 150 65 148 138 188; % starting point 
    190 175 128 122 244 73 163;% first via point  
    150 175 128 155 234 80 160; % first cube open
    150 175 128 155 234 80 205; % first cube close
    190 175 128 122 244 73 205;% second via point
    145 229 60  135 250 65 205; % deactivation station
    90  175 37  155 240 18 205; % third via point
    150 140 150 150 227 35 205; % drop off one open
    150 140 150 150 227 35 160 ; % drop off one close
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    190 175 128 122 244 73 163;%%%%%%%%% first via point
    150 200 97 148 234 80 160; % 2nd cube open
    150 200 97 148 234 80 205; % 2nd cube close
    190 175 128 122 244 73 205;% second via point
    145 229 60  135 250 65 205; % deactivation station
    90  175 37  155 240 18 205; % third via point %220 for 2nd
    150 130 150 150 221 30 205; % drop off one open
    150 130 150 150 221 30 160 ; % drop off one close
]% 20 each 
% Replace the line below with the Cubic or Quintic polynomial required to generate
% several points along a smooth path between every two via points in joint space.
% That is, you sample the Cubic or Quintic polynomial 10 times between the via
% points. This means that if you begin with 10 via points, you will end up with 91

Q = Q_via;

%%%%%%%%%%%%%%%%%%%%%% CALCULATE POINTS %%%%%%%%%%%%%%%%%%%%%%%%%
[points2, ~] = size(Q);
Qf = [];
for i=1:(points2-1)
    Qf = [Qf; poly(Q(i,:),Q(i+1,:))];
end    
Q = Qf
[points, ~] = size(Q); % Obtain the number of points to send

Robot = MSE4401BOT(1234,4321); % Create robot object

pause on; % Enable the use of pause command
% Prepare the robot to move, set to maximum torque and low speed 
disp('Ready to move the robot. Please press a key to continue...');
pause;
Robot.sendTorque([1700, 1700, 1700, 1700, 1700, 1700, 1700]);
Robot.sendSpeed([2, 2, 2, 2, 2, 2, 2]); % Can be increased up to 5 for each joint

% Sending the points to the robot
for i = 1:points
    Q_robot = Q(i,:); % Get the next point to be sent
    % Convert from robot angles to model angles
    % You should have a function like 'convertRobotToModel' from Lab 1
    % You may have to modify that function to accommodate the 7th gripper angle
    Q_model = convertRobotToModel(Q_robot);
    % Check for being close to singularity using the model, only first 6 angles
    J = ModelBot.jacob0(Q_model(1:6));
    DetJ = det(J);
    if abs(DetJ) < 0.0001
        Alarm_Singularity = 1;
    else
        Alarm_Singularity = 0;
    end
    
    % Check the joint limits
    Alarm_Limits = 0;
    for j = 1:7
        if Q_robot(j) > Q_UpperLimits(j) || Q_robot(j) < Q_LowerLimits(j)
            Alarm_Limits = 1;
        end
    end
    Alarm = Alarm_Limits + Alarm_Singularity; % Any Fault in the system
    % Send the point to the robot and wait for the robot to reach the point
    if Alarm == 0
        Robot.sendPosition(Q_robot);
        Q_current = transpose(Robot.getPosition);
        while norm(Q_current - Q_robot,2) > 5
            Q_current = transpose(Robot.getPosition);
        end
    else
        disp('The position is out of reach! Press enter to continue');
        pause;
        % Can do something like jumping over the point and sending the next one
    end
    %disp('Ready to move the robot. Please press a key to continue...');
    %pause;
end