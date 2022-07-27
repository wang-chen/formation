function catched = if_catched()
    global r1 r2 r3 d;
    if sqrt((r1.x-d.x)^2+(r1.y-d.y)^2)<=r1.k*d.v || sqrt((r2.x-d.x)^2+(r2.y-d.y)^2)<=r2.k*d.v || sqrt((r3.x-d.x)^2+(r3.y-d.y)^2)<=r3.k*d.v
        catched = true;
    else catched = false;
    end
end