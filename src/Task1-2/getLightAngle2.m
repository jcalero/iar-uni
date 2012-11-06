function direction = getLightAngle2(s)
angles = [170, 135, 95, 85, 45, 10, 300, 270];
%angles = deg2rad(angles);

sensorVals = readAmbient(s);

weights = (1 ./ sensorVals);
weights = weights .* 1/sum(weights)

direction = sum(angles .* weights);

end


