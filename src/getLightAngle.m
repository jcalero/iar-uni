function direction = getLightAngle(s)
angles = [170, 135, 95, 85, 45, 10, 300, 270];

% ALMOST THERE! Add each of the vectors..
vector1 = [cosd(angles(1)), sind(angles(1))]

vectors = [vector1];

%[x,y] = pol2cart(1, deg2rad(angles));
%vectors = [x;y]';

sensorVals = readAmbient(s);

weights = 1 ./ sensorVals;

weights = weights .* 1/sum(weights);

for i=1:size(vectors)
    vectors(i,:) = vectors(i,:) .* weights(i)
end

% Sum columns, not rows...
direction = sum(vectors,1);
