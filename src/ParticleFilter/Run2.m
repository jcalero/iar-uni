numparticles=20;    % number of particles
N=8;    % number of sensors
stepmove.speed=100;     % speed of movement
stepmove.turn=0.3;     % angle to turn at every iteration
load realmap
[particles, robot]=initialize2(numparticles,map);    % initializes particles and the robot

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
xPos = 460;
yPos = 490;

% go(s, 2);
% ===============

while true
   
    %[ robot.position, robot.direction ] = movement(robot.position, robot.direction,stepmove,map);   % performs the movement
    %[ sensor ] = measurement( robot.position , robot.direction, map,N );  % get new measurement
    %[ particles ] = ressample( particles, sensor, stepmove, map ,sigma);  % ressample
    
    % ================
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
    
    %stepmove.speed = abs(robot.position - oldrobot.position);
    
    %if (mod(i, 20) == 0)
        %fprintf('asdasda')
        [ sensor ] = 10200./readIR(s)';
        [ particles, bestPos ] = ressample2(particles, sensor, stepmove, map ,sigma);  % ressample
        
        %oldrobot.position = robot.position;
        
        plotparticles     % plots particles
        plot(robot.position(1), robot.position(2) ,'ro');    % plots actual position of the robot
        plot(bestPos(1), bestPos(2), 'rx');
    %end

    fprintf('paused. press any key to next iteration \n press ctrl+C to stop \n')
    pause(0.01);
end