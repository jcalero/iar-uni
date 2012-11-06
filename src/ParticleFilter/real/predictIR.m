function [ irVals ] = predictIR( pos,orient )
%PREDICTIR Summary of this function goes here
%   What this is supposed to do:
%   Calculate the positions and orientations of the ir sensors,
%   given the robot position and orientation.
%   Then use this information to get the distances

% TODO: Calculate the positions of the sensors
% TODO: Convert the ir values into distances

irVals = [50,50,50,50,50,50,50,50]; %in millimeters
sensorNum = 8;
sensorAngles = [170, 135, 95, 85, 45, 10, 300, 240];
sensorPositions = [[1, 3],[2,2.5],[3,1],[3,-1],[2,-2.5],[1,-3],[-3,-1],[-3,1]];

sensorAngles = sensorAngles + orient;

for i=1:sensorNum
    
    if sensorAngles(i) > 360
       sensorAngles(i) = sensorAngles(i) - 360;
    end
    
    [a b]=size(map.polyline);       %gets the number of polylines
    for j=1:b         %for all polylines
        for k=1:length(map.polyline{j}.p1)
            [ intersec, distan ] = distance( sensorPositions(i), sensorAngles(i), map.polyline{j}.p1(i,:)', map.polyline{j}.p2(i,:)' );
        
            %saves the smaller distance (closest objec that is in line with the sensor)
            if distan<irVals(n)
                irVals(n)=distan;            
            end
        end
    end
    
end

end

