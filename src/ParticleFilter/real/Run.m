numparticles=20;    % number of particles
N=8;    % number of sensors
stepmove.speed=0;     % speed of movement
stepmove.turn=0;     % angle to turn at every iteration
load realmap
[particles, robot]=initialize(numparticles,map);    % initializes particles and the robot

oldrobot.position = robot.position;
oldrobot.direction = robot.direction;

stddeviation=300;  % defines stddeviation. they are the same for every dimension.
ro=.5;   
sigma=[]; sigma(1:N,1:N)=ro*stddeviation^2; sigma=sigma-sigma.*eye(N)+eye(N).*stddeviation^2; % defines the covariance matrix 

axis equal
plotparticles     % plots particles
plot(robot.position(1), robot.position(2) ,'ro');    % plots actual position of the robot

% ===============
setCounts(s,0,0);
oldWheelCountsL = 0;
oldWheelCountsR = 0;
angle = 0;
xPos = 30;
yPos = 490;

go(s, 2);
% ===============

while true
    i = i + 1;
    
    % update Odometry
    % odo 1: Get wheel measurements
    wheelCounts = readCounts(s);
    wheelDeltaL = wheelCounts(1) - oldWheelCountsL;
    wheelDeltaR = wheelCounts(2) - oldWheelCountsR;
    oldWheelCountsL = wheelCounts(1);
    oldWheelCountsR = wheelCounts(2);

    % odo 2: calculate angle
    % φ ← φ + Δφ = φ - 0.5*(vleft - vright)/(widthfactor)
    % 2 x 40mm radius * 0.08mm per wheelcount != 'width factor' of 330, which I just found through trial and error
    angle = angle - 0.5*(wheelDeltaL - wheelDeltaR)/330;
    
    if(angle > 2*pi) 
        angle = angle - 2*pi; end
    if(angle < 0) 
        angle = angle + 2*pi; end
    
    % odo 3: calculate position
    % x ← x + Δx =x + 0.5*(vleft + vright) cos(φ)
    % y ← y + Δy =y + 0.5*(vleft + vright) sin(φ)
    xPos = xPos + 0.5*(wheelDeltaL + wheelDeltaR)*0.08 * cos(angle);
    yPos = yPos + 0.5*(wheelDeltaL + wheelDeltaR)*0.08 * sin(angle);

    % positions = cat(1,positions,[xPos,yPos]);
    
    robot.position = [xPos, yPos]';
    robot.direction = [cos(angle), sin(angle)]';
    
    turnangle = 0.5*(wheelDeltaL - wheelDeltaR)/330;
    stepmove.speed = mydist(robot.position, oldrobot.position);
    stepmove.turn = turnangle;
    
    stepmove
                        turnspeed = 2;
                            % read sensor values
                        ir = readIR(s);
                        % rename individual sensors
                        leftIR1 = ir(1);
                        leftIR2 = ir(2);
                        midLeftIR = ir(3);
                        midRightIR = ir(4);
                        rightIR2 = ir(5);
                        rightIR1 = ir(6);
    
                        % Adjust Left based on centre sensor
                        if (midRightIR > 150 || rightIR1 > 450 || rightIR2 > 600)
                            stop(s)
                            turn(s,-turnspeed,turnspeed)
                            direction = 'l';
                            turning = true;
                            continue
                        end

                        % Adjust Left based on centre sensor
                        if ((midLeftIR > 150 || leftIR1 > 450 || leftIR2 > 600))
                            stop(s)
                            turn(s,turnspeed,-turnspeed)
                            direction = 'r';
                            turning = true;
                            continue
                        end

                        % Wall re allignment
                        if (leftIR1 < 250 && leftIR1 > 180)
                            stop(s)
                            turn(s,-turnspeed,turnspeed)
                            direction = 'l';
                            turning = true;
                            continue
                        end

                        if (rightIR1 < 250 && rightIR1 > 180)
                            stop(s)
                            turn(s,turnspeed,-turnspeed)
                            direction = 'r';
                            turning = true;
                            continue
                        end
    
    %if (mod(i, 20) == 0)
        [ sensor ] = 10200./readIR(s)';
        bestPose = robot;
        if (stepmove.speed > 0 || stepmove.turn ~= 0)
            [ particles, bestPose ] = ressample(particles, sensor, stepmove, map ,sigma);  % ressample
        end
        
        oldrobot.position = robot.position;
        oldrobot.direction = robot.direction;
        
        plotparticles     % plots particles
        plot(robot.position(1), robot.position(2) ,'ro');    % plots actual position of the robot
        plot(bestPose.position(1), bestPose.position(2), 'r*');
    %end
                            go(s,2);
    fprintf('paused. press any key to next iteration \n press ctrl+C to stop \n')
    pause(0.01);
end