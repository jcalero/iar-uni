function [ weight ] = weights( particles, sensor, map ,sigma)
%WEIGHTS Gets the weights for the particles

[m,~]=size(particles.position);   % get the number of particles
weight=zeros(m,1);
for i=1:m
    weight(i) = probdensity(  particles.position(i,:)' , particles.direction(i,:)', map, sensor ,sigma);   
    %weights are proportional to the probability density of the real distace with the measurement 
end

weight=weight/norm(weight);       % then the vector of weights is normalized

end

