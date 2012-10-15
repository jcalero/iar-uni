function [angle,certainty] = getLightAngle(s)
  angles = [75, 0, 285, 210, 150];

  vector1 = [cosd(angles(1)), sind(angles(1))];
  vector2 = [cosd(angles(2)), sind(angles(2))];
  vector3 = [cosd(angles(3)), sind(angles(3))];
  vector4 = [cosd(angles(4)), sind(angles(4))];
  vector5 = [cosd(angles(5)), sind(angles(5))];


  vectors = [vector1; vector2; vector3; vector4; vector5];

  sensorVals = readAmbient(s);

  weights = 1 ./ sensorVals;

  weights = weights .* 1/sum(weights);

  vectors(1,:) = vectors(1,:) .* max(weights(1),weights(2));
  vectors(2,:) = vectors(2,:) .* max(weights(3),weights(4));
  vectors(3,:) = vectors(3,:) .* max(weights(5),weights(6));
  vectors(4,:) = vectors(4,:) .* weights(7);
  vectors(5,:) = vectors(5,:) .* weights(8);

  direction = sum(vectors,1);
  certainty = norm(direction);

  costheta = 
    dot(direction, [1,0])/(norm(direction)*norm([1,0]));
  angle = rad2deg(acos(costheta));

  if (direction(2) < 0)
      angle = -angle;
  end
end
