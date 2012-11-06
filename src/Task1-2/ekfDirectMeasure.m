function out = ekfDirectMeasure( estimatedX, estimatedY, irMapVals )
%EKFMEASURE Summary of this function goes here
%   Detailed explanation goes here

% irMapvals has to be an 8x2 matrix with the x,y values or each sensor
% so like: 
% 1 2 3 4 5 6 7 8
% 2 7 7 8 9 5 8 3

out = [];

for x = irMapVals
    out = [out pdist([x,[estimatedX;estimatedY]],'euclidean')];
end

end

