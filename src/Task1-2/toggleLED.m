function toggleLED(s,LED)
fprintf(s,['L,' num2str(LED) ',' num2str(2)]);
pause(0.001);
fscanf(s);
end