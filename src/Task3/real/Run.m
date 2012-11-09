addpath(genpath('.'));

numparticles=200;    % number of particles
N=8;    % number of sensors
stepmove.speed=0;     % speed of movement (distance to move at every iteration)
stepmove.turn=0;     % angle to turn at every iteration
load realmap
load distToIRMap
[particles, robot]=initialize(numparticles);    % initializes particles and the robot

oldrobot.position = robot.position;
oldrobot.direction = robot.direction;

stddeviation=300;  % defines stddeviation. they are the same for every dimension.
ro=.5;   
sigma=[]; sigma(1:N,1:N)=ro*stddeviation^2; sigma=sigma-sigma.*eye(N)+eye(N).*stddeviation^2; % defines the covariance matrix 

% Plot everything
plotparticles     % Particles
plot(robot.position(1), robot.position(2) ,'ro');    % Position of the robot
direc = robot.position+robot.direction*30;
plot([robot.position(1) direc(1)], [robot.position(2) direc(2)], 'k'); % Direction of the robot

fprintf('Initial state loaded. Is robot pose correct?\nPress any key to continue or CTRL+C to abort\n');
pause

% ===============
setCounts(s,0,0);
oldWheelCounts = [0 0];

go(s, 2);
%turn(s, -3, 1);
% ===============

while true
    % Generate new robot position and stepmove from odometry
    [ robot, stepmove, oldWheelCounts ] = odometry(s, oldrobot, oldWheelCounts);
    stepmove.turn
    % Read sensor data
    %[ sensor ] = 5100./readIR(s)';
    [ sensor ] = getDistFromIR(readIR(s), distToIRMap)';
    
    % Move particles and generate new predicted position
    bestPose = robot;
    if (stepmove.speed > 0 || stepmove.turn ~= 0)
        [ particles, bestPose ] = ressample(particles, sensor, stepmove, map ,sigma);  % ressample
    end
    bestPose.position
    
    % Update robot position with the predicted position
    if (size(bestPose.position, 1) == 1)
        fprintf('updating position');
        robot.position = bestPose.position;
        robot.direction = bestPose.direction;
    end

    % Save position for next iteration
    oldrobot.position = robot.position;
    oldrobot.direction = robot.direction;

    % Plot everything
    plotall(robot, particles, map, bestPose);
    
    % Do navigation & control here.
    % 

    fprintf('Paused. press any key for next iteration.\nPress ctrl+C to stop \n')
    pause(0.01);
end