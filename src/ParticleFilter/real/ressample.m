function [ particles, bestPose ] = ressample( particles, sensor, stepmove, map,sigma)
%RESSAMPLE Particle filter reassampling code. 
%   Detailed explanation goes here

reInitAmt = 0; % Percentage of samples that should be reinitialised.

[numpart ~]= size(particles.position);
for i=1:numpart
    [ pos, dir ] = moveParticles( particles.position(i,:)', particles.direction(i,:)', stepmove ,map);
    particles.position(i,:) =pos';   % move particles following the stepmove plan
    particles.direction(i,:)=dir';
end

weight  = weights( particles, sensor, map, sigma );    % get the weights for every particle
j = randsample(numpart,numpart,true,weight);    % draw particle index with replacement

if (find(weight == max(weight)) > 0)
    bestPose.position = particles.position(weight == max(weight), :);
    bestPose.direction = particles.direction(weight == max(weight), :);
end

posit=zeros(numpart,2);   %alocates position and direction matrices
direc=zeros(numpart,2);

for i=1:(numpart - floor(reInitAmt*numpart))
   
    posit(i,:) = particles.position(j(i),:) + randn(1,2)*5;   % get new particle, with drawn index and noise
    theta=randn/8;
    trans=[cos(theta) sin(theta);
           -sin(theta) cos(theta)];

    direc(i,:) = (trans*particles.direction(j(i),:)')';
end

if (reInitAmt > 0)
 [reinitpart, ~]=initializeRandom(floor(reInitAmt*numpart),map); % reinitializes 20 percent of the particles
 posit((numpart - floor(reInitAmt*numpart)+1):numpart,:)=reinitpart.position;
 direc((numpart - floor(reInitAmt*numpart)+1):numpart,:)=reinitpart.direction;
end


particles.position=posit;      %writes matrices on the structure
particles.direction=direc;


end
