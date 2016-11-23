%% /robot/joint_states parser
% author:  Andrew Simpson - ahfergus1@gmail.com
% purpose: Deal with getting Baxter's differing joint state messages into 
%          MATLAB to use with a controller.
% args: sig: message from Baxter
%       len: length of message (optional) assumed to be length of sig if
%            not given. Workaround for simulink not allowing variable
%            length arrays, lets it say how full the container array is.
%       i_pos: joint angles to initialize to (optional)
%       i_vel: joint speeds to initialize to (optional)
%       i_acl: joint effrts to initialize to (optional)
% rets: theta: current joint angles for left and right arm
%       omega: current joint speeds for left and right arm
%       alpha: current joint effrts for left and right arm
%       bJoints: update was for the arm joints (can be just grippers)
% TIP:
% If you don't understand the message format discussed here, run
% rostopic echo /robot/joint_states
% in a terminal while connected to the real or simulated Baxter.

function [theta,omega,effrt,bJoints] = joint_states(sig,len,i_pos,i_vel,i_eff)
%% Handle length
% internal length
if nargin >= 2
    len_i = len;
else
    len_i = length(sig.Position);
end

%% Decalre persistent (static) internal containers for the outputs
%  Old value will be held if joint not updated on this iteration
persistent theta_i
persistent omega_i
persistent effrt_i

%% Initialize persistent containers 
%  Use initializers if given
if nargin >= 3
    theta_i = i_pos;
end
if nargin >= 4
    omega_i = i_vel;
end
if nargin >= 5
    effrt_i = i_eff;
end
%  Initialize to zero if still empty
if isempty(theta_i)
    theta_i = zeros(14+2,1); % 14 joints + 2 grippers
end
if isempty(omega_i)
    omega_i = zeros(14+2,1); % just the arm joints get speeds
end
if isempty(effrt_i)
    effrt_i = zeros(14+2,1); % just the arm joints get efforts
end


% %% Reject messages for other joints
% % The physical Baxter sends the gripper states separately on the same
% % topic, so we have to reject it.  This is really annoying, they should 
% % have different topics or be in the same message.
% theta = zeros(14,1);
% omega = theta;
% alpha = theta;
% len = sig.Name_SL_Info.CurrentLength;
% if len < 14
%     bJoints = false;
%     return
% end

%% Search through data
%  My approach uses the length of the message to indicate its format. There
%  are three different formats on the /robot/joint_states channel right
%  now. Note that 1 and 17 both occur in the same session!
%  Length 1:
%       Real Baxter
%       'r_gripper_l_finger_joint' or 'l_gripper_l_finger_joint'
%       Note: I have never seen a X_gripper_r_finger_joint message on the
%       real Baxter.  It's not needed for the electric grippers.
%  Length 17:
%       Real Baxter
%       'head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1',
%       'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1','right_s0',
%       'right_s1', 'right_w0', 'right_w1', 'right_w2', and 'torso_t0'
%       Note: head_nod and torse_t0 are always 0 because our Baxter doesn't
%       have them.
%  Length 19:
%       Simulated Baxter
%       'head_pan', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint',
%       'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 
%       'left_w2', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint',
%       'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 
%       'right_w1', and 'right_w2'
%  My format:
%       'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 
%       'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 
%       'right_w0', 'right_w1', 'right_w2', 'l_gripper_l_finger_joint' 
%       and 'r_gripper_l_finger_joint'
switch len_i
    case 1
        % Get first char of the only name to know if it's left or right
        if iscell(sig.Name)
            side = sig.Name{1}(1);
        else
            side = char(sig.Name(1).Data(1));
        end
        if side == 'r'
            theta_i(end) = sig.Position(1);
            omega_i(end) = sig.Velocity(1);
            effrt_i(end) = sig.Effort(1);
        else
            theta_i(end-1) = sig.Position(1);
            omega_i(end-1) = sig.Velocity(1);
            effrt_i(end-1) = sig.Effort(1);
        end
        bJoints = false;
    case 17
        idxs = [5,6,3,4,7,8,9,12,13,10,11,14,15,16];
        theta_i = [sig.Position(idxs); theta_i(end-1:end)];
        omega_i = [sig.Velocity(idxs); omega_i(end-1:end)];
        effrt_i = [sig.Effort(idxs); effrt_i(end-1:end)];
        bJoints = true;
    case 19
        idxs = [6,7,4,5,8,9,10,15,16,13,14,17,18,19,2,11];
        theta_i = sig.Position(idxs);
        omega_i = sig.Velocity(idxs);
        effrt_i = sig.Effort(idxs);
        bJoints = true;
    otherwise
        disp('Invalid message length!');
        bJoints = false;
end

%% Output results
theta = theta_i;
omega = omega_i;
effrt = effrt_i;
return
end

%% OLD CODE - UNREACHABLE INTENTIONALLY - DELETE LATER
% %% Declare persistent (static) mapping between received messages and output
% persistent mapping
% 
% %% If first iteration to receive a valid message, build the mapping
% % The simulator and actual robot provide the joint states in the same
% % order, but the simulator includes the grippers and the actual provides
% % head pan, resulting in different lengths.  The messages have the joints
% % in alphabetical order, and the different messages come first, creating an
% % offset.
% % A string search could be done to build this table, but it isn't
% % neccessary for now.  I just mention it here
% if isempty(mapping)
% %     names={'left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',...
% %        'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2'};
%     
%     % More complicated mappings are possible, but for now I'll just use
%     % length to indicate what mapping I want. Note that we don't keep the
%     % messages in alphabetical order - shoulder-elbow-wrist is more
%     % intuitive.
%     
%     % Simulator mapping
%     mapping = [6,7,4,5,8,9,10,15,16,13,14,17,18,19];
%     if len == 17
%         % Change to physical mapping (one less preceeding joint)
%         mapping = mapping - 1;
%     end
% end
% 
% 
% %% Use mapping to get outputs
% for i=1:14
%     idx = mapping(i);
%     theta(i) = sig.Position(idx);
%     omega(i) = sig.Velocity(idx);
%     effrt(i) = sig.Effort(idx);
% end
