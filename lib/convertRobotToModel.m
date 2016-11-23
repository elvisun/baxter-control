function model_angles = convertRobotToModel( robot_angles )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

offset = [150 60 60 55 140 63 135];
model_angles = robot_angles-offset;

end

