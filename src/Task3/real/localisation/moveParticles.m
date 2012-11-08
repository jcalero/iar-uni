function [ position, direction ] = moveParticles( position, direction, stepmove ,map)
%MOVEMENT moves the robot or the particle following the 'stepmove' one step plan
%   'stepmove' is a one step plan with speed and turn angle
%   the function considers stochasticity affecting the turn angle and speed
%   and constraints the particle movements based on map data

theta=stepmove.turn+randn/8;      % the turn angle is afected by stochasticity
trans=[cos(theta) sin(theta);     % defines the rotation operator
      -sin(theta) cos(theta)];
speed=stepmove.speed+randn*10;    %the speed is afected by stochasticity

direction= trans*direction;         %rotates the direction
direction=direction/norm(direction); %normalizes the direction

mindistance=inf;    % initilizes the minimum distance with a high value
[~, b]=size(map.polyline);   % gets the number of polylines in the map
for j=1:b       % for all polylines
    for i=1:length(map.polyline{j}.p1)        %for all segments
        % searches for possible intersection, representing objects that
        % could block the movement
        [ ~, distan ] = distance( position, direction, map.polyline{j}.p1(i,:)', map.polyline{j}.p2(i,:)' );
        if distan<mindistance
            mindistance=distan;            
        end
    end
end

mindistance=speed;

% if the intersection is close enought, the stops before the
% intersection
if  mindistance<stepmove.speed
    position=position+direction*(mindistance-5);
else
    position=position+direction*speed;  %if there is no intersection close enought, it moves the whole step
end
    
end

