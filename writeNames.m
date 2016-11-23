function [ msg ] = writeNames( names, msg )
%writeNames Writes joint names into baxter/joint_command ROS message.
%   Writing names into a ROS message in Simulink is more complicated than
%   it should be. This function handles it in one place to simplify the
%   Simulink model.

msg.Names_SL_Info.CurrentLength=uint32(length(names));
for idx=1:length(names) % Iterate through each component
    % Each data field is a string.
    % The following is a bit of a hack since Simulink handles strings in a
    % weird way. Simulink only supports fixed-size arrays, so each string
    % is put into a buffer and then the actual length is put into the
    % current length member.
    str=names{idx};
    strLen=length(str);
    msg.Names(idx).Data(1:strLen)=uint8(str); 
    msg.Names(idx).Data_SL_Info.CurrentLength=uint32(strLen);
end

end

