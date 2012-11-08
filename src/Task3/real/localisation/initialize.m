function [particles, robot] = initialize( numpartic )
%INITIALIZE initilizes random particles and the position and direction for
%the robot

fprintf('Initializing particles and the robot based on starting position\n')

% Start position and direction
startX = 30;
startY = 490;
startA = [cos(0); sin(0)];

% initialises particles at the starting position and direction
particles.position=[(repmat(startX, numpartic, 1)) (repmat(startY, numpartic, 1))];
particles.direction = repmat(startA', numpartic, 1);

% puts the robot at the start position
robot.position = [startX startY]';
robot.direction = startA;

end