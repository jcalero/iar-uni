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

while (true)
	irSens = readIR(s);
    irR1 = irSens(6);
    irR2 = irSens(5);
    irL1 = irSens(1);
    irL2 = irSens(2);
    irC1 = irSens(3);
    irC2 = irSens(4);
    
    lSpeed = 6;
    rSpeed = 6;
	
	if(irR1 > 200)
		lSpeed = lSpeed - round(irR1/100);
	end
    
	if(irL1 > 200)
		rSpeed = rSpeed - round(irL1/100);
	end
    
    
    lastIrL1 = irL1;
    lastIrR1 = irR1;
    
    
    if(irC1 > 200 || irC2 > 200)
       lSpeed = lSpeed - ((irC1 - irC2)/(abs(irC1 - irC2))) * round(irC1/100)
       rSpeed = rSpeed - ((irC1 - irC2)/(abs(irC1 - irC2))) * round(irC2/100)
    end
    
    if(lSpeed > 6) lSpeed = 6; end
    if(rSpeed > 6) rSpeed = 6; end
    
    setSpeeds(s,lSpeed,rSpeed);
	pause(0.05);
end
stop(s);

%%%%%%%%%%%%%%%

%% CLEANUP %%%
%closeConnection(s);

%%%%%%%%%%%%%%
