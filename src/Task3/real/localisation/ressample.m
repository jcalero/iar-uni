function [ particles, bestPose ] = ressample( particles, sensor, stepmove, map, sigma)
%RESSAMPLE Particle filter reassampling code. 
%   Detailed explanation goes here

load centerPointsMap

reInitAmt = 0; % Percentage of samples that should be reinitialised.

[numpart ~]= size(particles.position);
tic
for i=1:numpart
    [ pos, dir ] = moveParticles( particles.position(i,:)', particles.direction(i,:)', stepmove, map, centerPoints);
    particles.position(i,:)  = pos';   % move particles following the stepmove plan
    particles.direction(i,:) = dir';
end
toc

%timerWeights = tic;
weight = weights( particles, sensor, map, sigma );    % get the weights for every particle
%wgt = toc(timerWeights);
j = randsample(numpart,numpart,true,weight);    % draw particle index with replacement

if (find(weight == max(weight)) > 0)
    bestPose.position = particles.position(weight == max(weight), :);
    bestPose.direction = particles.direction(weight == max(weight), :);
end

%alocates position and direction matrices
posit=zeros(numpart,2);
direc=zeros(numpart,2);

for i=1:(numpart - floor(reInitAmt*numpart))
    posit(i,:) = particles.position(j(i),:);   % get new particle, with drawn index
    direc(i,:) = particles.direction(j(i),:);
end

if (reInitAmt > 0)
    [reinitpart, ~] = initializeRandom(floor(reInitAmt*numpart),map); % reinitializes 'reInitAmt' percent of the particles
    posit((numpart - floor(reInitAmt*numpart)+1):numpart,:)=reinitpart.position;
    direc((numpart - floor(reInitAmt*numpart)+1):numpart,:)=reinitpart.direction;
end


particles.position=posit;      %writes matrices on the structure
particles.direction=direc;


end
