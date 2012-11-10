addpath(genpath('.'));

numparticles=200;    % number of particles
N=8;                 % number of sensors
stepmove.speed=0;    % speed of movement (distance to move at every iteration)
stepmove.turn=0;     % angle to turn at every iteration
t = 0;
load realmap
load distToIRMap
load centerPointsMap
potFieldMap = getPotFieldMap();

goals = [450 150];

[particles, robot]=initialize(numparticles);    % initializes particles and the robot

oldrobot.position = robot.position;
oldrobot.direction = robot.direction;

stddeviation=300;  % defines stddeviation. they are the same for every dimension.
ro=.5;   
sigma=[]; sigma(1:N,1:N)=ro*stddeviation^2; sigma=sigma-sigma.*eye(N)+eye(N).*stddeviation^2; % defines the covariance matrix 

% Initial plotting
plotall(robot, particles, map, robot);

fprintf('Initial state loaded. Is robot pose correct?\nPress any key to continue or CTRL+C to abort\n');
pause

% ===============
setCounts(s,0,0);
oldWheelCounts = [0 0];

%go(s, 2);
% ===============

speed = 6;
turnspeed = speed/2;
lSpeed = speed;
rSpeed = speed;
oldLSpeed = speed;
oldRSpeed = speed;

while true
    
    t = t + 1;
    
    % Generate new robot position and stepmove from odometry
    [ robot, stepmove, oldWheelCounts ] = odometry(s, oldrobot, oldWheelCounts);
    
    % Read sensor data
    %[ sensor ] = 5100./readIR(s)';
    [ sensor ] = getDistFromIR(readIR(s), distToIRMap)';
    leftIR1 = sensor(1);
    leftIR2 = sensor(2);
    midLeftIR = sensor(3);
    midRightIR = sensor(4);
    rightIR2 = sensor(5);
    rightIR1 = sensor(6);
    backRightIR = sensor(7);
    backLeftIR = sensor(8);
    
    % Move particles and generate new predicted position
    bestPose = robot;
%     bestPose.position = bestPose.position';
%     bestPose.direction = bestPose.direction';
    if ((stepmove.speed > 0 || stepmove.turn ~= 0))
        [numpart ~]= size(particles.position);
        for i=1:numpart
            [ pos, dir ] = moveParticles( particles.position(i,:)', particles.direction(i,:)', stepmove, map, centerPoints);
            particles.position(i,:)  = pos';   % move particles following the stepmove plan
            particles.direction(i,:) = dir';
        end
        if (mod(t, 10) == 0)
            stop(s);
            [ particles, bestPose ] = ressample(particles, sensor, stepmove, map ,sigma);  % ressample
            setSpeeds(s, lSpeed, rSpeed);
        end
    end
    size(bestPose.position);
    
    bestPose.position(1,:);
    
    % Update robot position with the predicted position
    if (size(bestPose.position, 1) == 1 || size(bestPose.position, 2) == 1)
        %fprintf('here\n');
        %robot.position = bestPose.position;
        %robot.direction = bestPose.direction;
    else
        %fprintf('there\n');
        %robot.position = bestPose.position(1,:);
        %robot.direction = bestPose.direction(1,:);
    end
    
    if (size(robot.position, 1) == 2 && size(robot.position, 2) == 1)
        robot.position = robot.position';
        robot.direction = robot.direction';
    end

    % Save position for next iteration
    oldrobot.position = robot.position;
    oldrobot.direction = robot.direction;

    % Plot everything
    plotall(robot, particles, map, bestPose);
    
    % Do navigation & control here.
    
    foundFood = 0;
    indeces = size(find(sensor > 30),1);
    if indeces == 0
       foundFood = 1;
    end
    goals
    % && (size(goals,1) == 0 || size(strcmp([ 1 1 ], ismember(goals,robot.position))) > 0)
    if (foundFood == 1 && (size(strmatch([ 1 1 ], ismember(goals,robot.position+1)), 1) == 0))
        fprintf('FOOOOOOOOOOOOD! Nom nom nom nomonomomomnnom....\n');
        if (mydist(robot.position, goals(size(goals,1),:)) > 20)
            goals = [goals; robot.position];
        end
        stop(s);
        continue
    end
    
    
    [a,c] = getPotFieldVec(robot, potFieldMap, goals);
    
    a
    
    if((rightIR1 < 37.5 || rightIR2 < 28) && (leftIR1 < 37.5 || leftIR2 < 28) && ((midRightIR + midLeftIR)/2 > 55))
        go(s,speed);
        continue;
    end
    
    % Adjust Left based on centre sensor
    if (midRightIR < 55 || rightIR1 < 37.5 || rightIR2 < 28)
        stop(s)
        turn(s,-turnspeed,turnspeed)
        continue
    end

    % Adjust Left based on centre sensor
    if ((midLeftIR < 55 || leftIR1 < 37.5 || leftIR2 < 28))
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