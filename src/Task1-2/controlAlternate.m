function positions = controlAlternate(s,speed)

%%% INIT %%%
% s = openConnection;
% setCounts(s,0,0);
% ir_read = readIR(s);
% ir_l1 = ir_read(1);
% ir_l2 = ir_read(2);
% ir_cl = ir_read(3);
% ir_cr = ir_read(4);
% ir_r2 = ir_read(5);
% ir_r1 = ir_read(6);
%%%%%%%%%%%%

%%% CONTROL %%%

% go(s,5);
% d = readIR(s);
% d = d(3);
irSens = readIR(s);
lastIrR1 = irSens(6);
lastIrL1 = irSens(1);

setCounts(s,0,0);
oldWheelCountsL = 0;
oldWheelCountsR = 0;
angle = 0;
xPos = 0;
yPos = 0;

positions = [xPos,yPos];

iteration = 0;

while (iteration < 1000)
    
    iteration = iteration +1;
    
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

    positions = cat(1,positions,[xPos,yPos]);
    
	irSens = readIR(s);
    irR1 = irSens(6);
    irR2 = irSens(5);
    irL1 = irSens(1);
    irL2 = irSens(2);
    irC1 = irSens(3);
    irC2 = irSens(4);
    
    lSpeed = speed;
    rSpeed = speed;
	
	if(irR1 > 200)
		lSpeed = lSpeed - round(irR1/100);
	end
    
	if(irL1 > 200)
		rSpeed = rSpeed - round(irL1/100);
	end
    
    
    lastIrL1 = irL1;
    lastIrR1 = irR1;
    
    
    if(irC1 > 200 || irC2 > 200)
       lSpeed = lSpeed - ((irC1 - irC2)/(abs(irC1 - irC2))) * round(irC1/100);
       rSpeed = rSpeed - ((irC1 - irC2)/(abs(irC1 - irC2))) * round(irC2/100);
    end
    
    if(lSpeed > 6) 
        lSpeed = 6; end
    if(rSpeed > 6) 
        rSpeed = 6; end
    
    setSpeeds(s,lSpeed,rSpeed);
	pause(0.05);
end
stop(s);

%%%%%%%%%%%%%%%

%% CLEANUP %%%
%closeConnection(s);

%%%%%%%%%%%%%%

end
