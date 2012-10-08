function angle = getLightAngleSimple(s)
    sensorVals = readAmbient(s);

    lowestValue = find(sensorVals==min(min(sensorVals)));

    switch (lowestValue(1))
        case 1
            angle = 170;
        case 2
            angle = 135;
        case 3
            angle = 95;
        case 4
            angle = 85;
        case 5
            angle = 45;
        case 6
            angle = 10;
        case 7
            angle = 300;
        case 8
            angle = 270;
    end
end
