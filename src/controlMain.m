function controlMain(s, speed)
    turnspeed = round(speed/2);
    
    setCounts(s,0,0);
    oldWheelCountsL = 0;
    oldWheelCountsR = 0;
    angle = 0;
    xPos = 0;
    yPos = 0;

    while (true)
        % read sensor values
        ir = readIR(s);
        % rename individual sensors
        leftIR1 = ir(1);
        leftIR2 = ir(2);
        midLeftIR = ir(3);
        midRightIR = ir(4);
        rightIR2 = ir(5);
        rightIR1 = ir(6);
        
        % update Odometry
        % odo 1: Get wheel measurements
        wheelCounts = readCounts(s);
        wheelDeltaL = wheelCounts(1) - oldWheelCountsL;
        wheelDeltaR = wheelCounts(2) - oldWheelCountsR;
        oldWheelCountsL = wheelCounts(1);
        oldWheelCountsR = wheelCounts(2);
        
        % odo 2: calculate angle
     	% φ ← φ + Δφ = φ - 0.5*(vleft - vright)/(widthfactor)
     	% 2 x 40mm radius * 0.08mm per wheelcount != "width factor" of 330, which I just found through trial and error
     	angle = angle - 0.5*(wheelDeltaL - wheelDeltaR)/330;
     	
     	if(angle > 2*pi) angle = angle - 2*pi; end
     	if(angle < 0) angle = angle + 2*pi; end
        
        
        % odo 3: calculate position
        % x ← x + Δx =x + 0.5*(vleft + vright) cos(φ)
		% y ← y + Δy =y + 0.5*(vleft + vright) sin(φ)
		xPos = xPos + 0.5*(wheelDeltaL + wheelDeltaR)*0.08 * cos(angle)
		yPos = yPos + 0.5*(wheelDeltaL + wheelDeltaR)*0.08 * sin(angle)
		

        % Adjust Left based on centre sensor
        if (midRightIR > 300 || rightIR1 > 500)
            stop(s)
            turn(s,-turnspeed,turnspeed)
            continue
        end

        % Adjust Left based on centre sensor
        if (midLeftIR > 300 || leftIR1 > 500)
            stop(s)
            turn(s,turnspeed,-turnspeed)
            continue
        end

        % Wall re allignment
        if (leftIR1 < 250 && leftIR1 > 180)
            stop(s)
            turn(s,-turnspeed,turnspeed)
            continue
        end

        if (rightIR1 < 250 && rightIR1 > 180)
            stop(s)
            turn(s,turnspeed,-turnspeed)
            continue
        end

        go(s,speed);

        pause(0.05);
    end
end

