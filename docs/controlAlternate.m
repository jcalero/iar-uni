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
    lSpeed = lSpeed - ((irC1 - irC2)/(abs(irC1 - irC2)))
      * round(irC1/100)
    rSpeed = rSpeed - ((irC1 - irC2)/(abs(irC1 - irC2)))
      * round(irC2/100)
  end
  
  if(lSpeed > 6) lSpeed = 6; end
  if(rSpeed > 6) rSpeed = 6; end
  
  setSpeeds(s,lSpeed,rSpeed);
	pause(0.05);
end
stop(s);
