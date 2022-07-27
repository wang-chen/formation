clc
clear
close all
global r1 r2 r3 d;

%speed and initial position of the evader
d.x = -1;    d.y=0;     
d.v = 0.7;    

%speed ratio and initial position of the pursuer
r1.x = -10.9;  r1.y = 13.2;      r1.k = 0.95;  
r2.x = -10.9;  r2.y = -10.9;     r2.k = 0.95; 
r3.x = 10.9;   r3.y = -0.9;      r3.k = 0.95; 


figure,hold on,grid off
title('Example');
 
catched=false;

while ~catched
    flee(); % strategy of evader
    hunt(); % strategy of pursuer
    catched = if_catched();
    plots();
    pause(0.001);
end
