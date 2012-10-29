function [newXPos,newYPos,newAngle] = ekf(xPos,yPos,angle)
%EKF extended kalman filter
%   filters kalmanily

translationalNoise = 0.5; % just random values for testing
rotationalNoise = 0.5; % just random values for testing

%wheelCounts = readCounts(s);
wheelCounts = [200 100];% just random values for testing
oldWheelCountsL = 0;% just random values for testing
oldWheelCountsR = 0;% just random values for testing
wheelDeltaL = wheelCounts(1) - oldWheelCountsL;
wheelDeltaR = wheelCounts(2) - oldWheelCountsR;
odoDist = ((wheelDeltaL + wheelDeltaR)/2); % *T (what the fuck is 'T'?? don't think we actually need it)

odoAngle = angle - 0.5*(wheelDeltaL - wheelDeltaR)/330;
if(odoAngle > 2*pi) odoAngle = odoAngle - 2*pi; end
if(odoAngle < 0) odoAngle = odoAngle + 2*pi; end

jacobianMatrix = [(-odoDist * sin(odoAngle)) cos(2*angle);(odoDist * cos(odoAngle)) sin(2*angle); 1 0];
jacobianMatrixDash = [1 0 (-odoDist * sin(odoAngle)); 0 1 (odoDist * cos(odoAngle)); 0 0 1];

noiseMatrix = [((angle^2)*rotationalNoise) 0; 0 (translationalNoise)];

noiseCovMatrix = jacobianMatrix * noiseMatrix * transpose(jacobianMatrix);

oldErrorCovMatrix = ones(3,3); % this still needs to be initialized properly somewhere at the first timestep. just here for testing purposes
errorCovMatrix = jacobianMatrixDash * oldErrorCovMatrix * transpose(jacobianMatrixDash) + noiseCovMatrix

% stuck at trying to calculate P(k + 1|k), which is defined using itself...


end

