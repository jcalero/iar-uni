addpath(genpath('.'));

numparticles=20;    % number of particles
N=8;    % number of sensors
stepmove.speed=0;     % speed of movement (distance to move at every iteration)
stepmove.turn=0;     % angle to turn at every iteration
load realmap
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

%go(s, 2);
turn(s, 3, -1);
% ===============

while true
    % Do navigation & control here.
    % 
    
    % Generate new robot position and stepmove from odometry
    [ robot, stepmove, oldWheelCounts ] = odometry(s, oldrobot, oldWheelCounts);

    % Read sensor data
    [ sensor ] = 10200./readIR(s)';
    
    % Move particles and generate new predicted position
    bestPose = robot;
    if (stepmove.speed > 0 || stepmove.turn ~= 0)
        [ particles, bestPose ] = ressample(particles, sensor, stepmove, map ,sigma);  % ressample
    end
    
    % Update robot position with the predicted position
    % robot.position = bestPose.position;
    % robot.direction = bestPose.direction;

    % Save position for next iteration
    oldrobot.position = robot.position;
    oldrobot.direction = robot.direction;

    % Plot everything
    plotall(robot, particles, map, bestPose);

    fprintf('Paused. press any key for next iteration.\nPress ctrl+C to stop \n')
    pause(0.01);
    pause;
end