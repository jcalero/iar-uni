function [ outVector ] = getPotFieldVec( roboPos,map,food )
%GETPOTFIELDVEC Summary of this function goes here
%   Detailed explanation goes here

outVector = [0 0];

% go through entire map of objects, calculate distance to objects,
% check if distance within 10cm, calculate vector from object to robot,
% add to overall vector
s = size(map);
for x = map'
    d = sqrt(sum((x'-roboPos).^2,2));
    if d < 100
        temp = (roboPos - x')/norm((roboPos - x'));
        outVector = outVector + ((100 - d)/(100*s(1)) .* temp);
    end 
end


% go through entire map of food sources, calculate distance,
% check if distance within 100cm, calculate vector from robot to food,
% add to overall vector
s = size(food);
for y = food
    
    d = sqrt(sum((y'-roboPos).^2,2));
    if(d < 1000)
        temp = (y' - roboPos)/norm((y' - roboPos));
        outVector = outVector + ((1000 - d)/(1000*s(1)).* temp);
    end
end

end

