function [ oldLSpeed, oldRSpeed, foods, target ] = control(s, robot, potFieldMap, sensor, foods, target, speed, turnspeed, oldLSpeed, oldRSpeed )
%CONTROL Summary of this function goes here
%   Detailed explanation goes here

    home = [ 460 490 ];

    % Rename sensor values
    leftIR1 = sensor(1);
    leftIR2 = sensor(2);
    midLeftIR = sensor(3);
    midRightIR = sensor(4);
    rightIR2 = sensor(5);
    rightIR1 = sensor(6);
    %backRightIR = sensor(7);
    %backLeftIR = sensor(8);
    
    if (mydist(home, robot.position) < 20 && isequal(target,home))
        fprintf('Robot is close to home\n');
        if (size(foods,1) > 1)
            fprintf('New target set to first food source\n');
            target = foods(1, :);
        else
            fprintf('New target set randomly\n');
            target = [rand*940 rand*1440];
        end
    end
    
    if (mydist(target, robot.position) < 20)
        fprintf('New target set randomly because reached target\n');
        target = [rand*940 rand*1440];
    end
    
    plot(target(1), target(2), 'og');
    
    % Check if we found food
    foundFood = 0;
    indeces = size(find(sensor > 30),1);
    if indeces == 0
       foundFood = 1;
    end
    
    % Do stuff when we found food.
    if (foundFood == 1 && (size(strmatch([ 1 1 ], ismember(foods,robot.position)), 1) == 0))
        fprintf('FOOOOOOOOOOOOD! Nom nom nom nomonomomomnnom....\n');
        stop(s);
        inGoals = 0;
        for i=1:size(foods,1)
            if (mydist(robot.position, foods(i,:)) < 20)
                inGoals = 1;
                break;
            end
        end
        
        if (~inGoals)
            foods = [foods; robot.position];
            target = home;
        end
        return
    end

    % Baseline obstacle avoidance code
    % =======================
    if((rightIR1 < 37.5 || rightIR2 < 28) && (leftIR1 < 37.5 || leftIR2 < 28) && ((midRightIR + midLeftIR)/2 > 55))
        go(s,speed);
        return
    end
    
    % Adjust Left based on centre sensor
    if (midRightIR < 55 || rightIR1 < 37.5 || rightIR2 < 28)
        stop(s)
        turn(s,-turnspeed,turnspeed)
        return
    end

    % Adjust Left based on centre sensor
    if ((midLeftIR < 55 || leftIR1 < 37.5 || leftIR2 < 28))
        stop(s)
        turn(s,turnspeed,-turnspeed)
        return
    end
    % ======================
    
    % Get potential field direction
    [a,c] = getPotFieldVec(robot, potFieldMap, target);
        
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

end

