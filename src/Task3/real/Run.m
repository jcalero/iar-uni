addpath(genpath('.'));

numparticles=200;    % number of particles
N=8;                 % number of sensors
stepmove.speed=0;    % speed of movement (distance to move at every iteration)
stepmove.turn=0;     % angle to turn at every iteration
load realmap
load distToIRMap

goals = [];

[particles, robot]=initialize(numparticles);    % initializes particles and the robot

oldrobot.position = robot.position;
oldrobot.direction = robot.direction;

stddeviation=300;  % defines stddeviation. they are the same for every dimension.
ro=.5;   
sigma=[]; sigma(1:N,1:N)=ro*stddeviation^2; sigma=sigma-sigma.*eye(N)+eye(N).*stddeviation^2; % defines the covariance matrix 

x = robot.position(1);
y = robot.position(2);
r = 70;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);

% Plot everything
plotparticles     % Particles
plot(robot.position(1), robot.position(2) ,'ro');    % Position of the robot
plot(x+xp,y+yp);
direc = robot.position+robot.direction*30;
plot([robot.position(1) direc(1)], [robot.position(2) direc(2)], 'k'); % Direction of the robot

fprintf('Initial state loaded. Is robot pose correct?\nPress any key to continue or CTRL+C to abort\n');
pause

% ===============
setCounts(s,0,0);
oldWheelCounts = [0 0];

go(s, 2);
% ===============

speed = 2;
turnspeed = speed/2;
lSpeed = speed;
rSpeed = speed;
oldLSpeed = speed;
oldRSpeed = speed;

while true
    
    % Generate new robot position and stepmove from odometry
    [ robot, stepmove, oldWheelCounts ] = odometry(s, oldrobot, oldWheelCounts);
    
    % Read sensor data
    %[ sensor ] = 5100./readIR(s)';
    [ sensor ] = getDistFromIR(readIR(s), distToIRMap)';
    
    % Move particles and generate new predicted position
    bestPose = robot;
    if (stepmove.speed > 0 || stepmove.turn ~= 0)
        [ particles, bestPose ] = ressample(particles, sensor, stepmove, map ,sigma);  % ressample
    end
    size(bestPose.position)
    
    % Update robot position with the predicted position
    if (size(bestPose.position, 1) == 1)
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
    
    foundFood = 0;
    [~,indeces] = size(find(sensor > 30));
    if indeces > 0
       foundFood = 1; 
    end
    
    if foundFood && find(ismember(goals,robot.position)) > 0
        goals = [goals robot.position];
        stop(s);
        continue
    end
    
    [a,c] = getPotFieldVec(robot.position,robot.direction,realMap,goals);
    
    if((rightIR1 > 250 || rightIR2 > 400) && (leftIR1 > 250 || leftIR2 > 400) && ((midRightIR + midLeftIR)/2 < 150))
        setSpeeds(s,speed);
        continue;
    end
    
    % Adjust Left based on centre sensor
    if (midRightIR > 150 || rightIR1 > 250 || rightIR2 > 400)
        stop(s)
        turn(s,-turnspeed,turnspeed)
        continue
    end

    % Adjust Left based on centre sensor
    if ((midLeftIR > 150 || leftIR1 > 250 || leftIR2 > 400))
        stop(s)
        turn(s,turnspeed,-turnspeed)
        continue
    end
    
    if(a > 0)
        rSpeed = speed;
        lSpeed = max(round(speed - (12 * abs(a) / 180) -1),-speed);
    else
        lSpeed = speed;
        rSpeed = max(round(speed - (12 * abs(a) / 180) -1),-speed);
    end
    
    
    if(oldLSpeed ~= lSpeed || oldRSpeed ~= rSpeed)
         setSpeeds(s,lSpeed,rSpeed);
         oldLSpeed = lSpeed;
         oldRSpeed = rSpeed;
    end

    fprintf('Paused. press any key for next iteration.\nPress ctrl+C to stop \n')
    pause(0.01);
end