function [ angle, intensity ] = getPotFieldVec( roboPos,roboDirection,map,food )
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
        factor = 400/(d^2);
        outVector = outVector + (factor .* temp);
    end
end

% go through entire map of food sources, calculate distance,
% check if distance within 100cm, calculate vector from robot to food,
% add to overall vector
s = size(food);
for y = food'
    d = sqrt(sum((y'-roboPos).^2,2));
    if(d < 2000)
        temp = (y' - roboPos)/norm((y' - roboPos));
        outVector = outVector + ((2000 - d)/(2000*s(1)).* temp);
    end
end

intensity = norm(outVector);
outVector = outVector/norm(outVector);

costheta = dot(outVector, roboDirection)/(norm(outVector)*norm(roboDirection));
angle = rad2deg(acos(costheta));

if roboDirection(1)*outVector(2) - roboDirection(2)*outVector(1)< 0
    angle = -angle;
end

end

