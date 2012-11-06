numparticles=50;    % number of particles
N=5;    % number of sensors
stepmove.speed=100;     % speed of movement
stepmove.turn=0.3;     % angle to turn at every iteration
load realmap
[particles, robot]=initializeSim(numparticles,map);    % initializes particles and the robot

stddeviation=300;  % defines stddeviation. they are the same for every dimension.
ro=.5;   
sigma=[]; sigma(1:N,1:N)=ro*stddeviation^2; sigma=sigma-sigma.*eye(N)+eye(N).*stddeviation^2; % defines the covariance matrix 

plotparticles     % plots particles
plot(robot.position(1), robot.position(2) ,'ro');    % plots actual position of the robot

while true
   
    [ robot.position, robot.direction ] = movement(robot.position, robot.direction,stepmove,map);   % performs the movement
    [ sensor ] = measurement( robot.position , robot.direction, map,N );  % get new measurement
    [ particles, bestPos ] = ressample( particles, sensor, stepmove, map ,sigma);  % ressample
    
    plotparticles     % plots particles
    plot(robot.position(1), robot.position(2) ,'ro');    % plots actual position of the robot

    direc = [robot.position(:,1) robot.position + robot.direction*50];
    plot(direc(1,:),direc(2,:),'r-');
    plot(bestPos(1), bestPos(2), 'r*');

    fprintf('paused. press any key to next iteration \n press ctrl+C to stop \n')
    pause
end