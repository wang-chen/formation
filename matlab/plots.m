function plots()

    global r1 r2 r3 d;
    
    axis equal;
    axis ([-20,20,-20,20]);
    hold on;
    plot(d.x,d.y,'ko');
    plot(r1.x,r1.y,'r*'); 
    plot(r2.x,r2.y,'g*');
    plot(r3.x,r3.y,'b*');
    
    legend('E1','P1','P2','P3');
    xlabel('X(m)');
    ylabel('Y(m)');
    
    
end 