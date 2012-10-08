function goToPoint(s,posLeft,posRight)
fprintf(s,['C,' num2str(posLeft) ',' num2str(posRight)]);
pause(0.001);
fscanf(s);
end
