
function flee()

    global r1 r2 r3 d;    
    
    d1 = sqrt((r1.x-d.x)^2+(r1.y-d.y)^2);
    k1 = 1/d1; % repulsive parameter
    v1x=k1*(d.x-r1.x)/d1;
    v1y=k1*(d.y-r1.y)/d1;
  
    d2 = sqrt((r2.x-d.x)^2+(r2.y-d.y)^2);
    k2 = 1/d2; % repulsive parameter
    v2x=k2*(d.x-r2.x)/d2;
    v2y=k2*(d.y-r2.y)/d2;
  
    d3 = sqrt((r3.x-d.x)^2+(r3.y-d.y)^2);
    k3 = 1/d3; % repulsive parameter
    v3x=k3*(d.x-r3.x)/d3;
    v3y=k3*(d.y-r3.y)/d3;
  
    vtx = v1x+v2x+v3x;
    vty = v1y+v2y+v3y;
    
    % destination of evader
    pe = [-50 2];
    
    % ideal moving direction to its destination
    kx = pe(1)/sqrt(pe(1)^2+pe(2)^2); ky = pe(2)/sqrt(pe(1)^2+pe(2)^2);
    
    %  combine escape direction and ideal moving direction
    vx = vtx/sqrt(vtx^2+vty^2) + kx;
    vy = vty/sqrt(vtx^2+vty^2) + ky;
    
    % you can design different strategies of the evader. 
    
    d.x = d.v*vx/sqrt(vx^2+vy^2)+d.x;
    d.y = d.v*vy/sqrt(vx^2+vy^2)+d.y;

end