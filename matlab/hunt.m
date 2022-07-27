function hunt()

global r1 r2 r3 d;

 [fa1, r11]=cart2pol(r1.x-d.x,r1.y-d.y);
 [fa2, r12]=cart2pol(r2.x-d.x,r2.y-d.y);
 [fa3, r13]=cart2pol(r3.x-d.x,r3.y-d.y);


 if fa1<0
     fa1=fa1+2*pi;
 end
    
 if fa2<0
     fa2=fa2+2*pi;
 end
    
 if fa3<0
    fa3=fa3+2*pi;
 end

 ff=[fa1,fa2,fa3];

 fmin= min(ff);
 fmax= max(ff);
 fmidd= median(ff);

 vi1 = d.v*r1.k;
 vi2 = d.v*r2.k;
 vi3 = d.v*r3.k;

 % strategy of pursuer 1

  if fa1==fmin
     ke1 = fmax+fmidd-2*pi-2*fmin;

  elseif fa1==fmidd
     ke1 = fmax+fmin-2*fmidd;
   
  else 
     ke1 = 2*pi+fmin+fmidd-2*fmax;
  end

  m1 = -abs(ke1)/(4*pi-asin(r2.k)+asin(r3.k))*sin(pi*((r11)/(r11+r12+r13))^(log(2)/log(3)));
  beta1 = pi/2*(1-exp(m1));
    
  vrx1= -vi1*sin(beta1)*sign(ke1)*sin(fa1)-vi1*cos(beta1)*cos(fa1);
  vry1= vi1*sin(beta1)*sign(ke1)*cos(fa1)-vi1*cos(beta1)*sin(fa1);

  r1.x=r1.x+ vrx1;
  r1.y=r1.y+ vry1;


  % strategy of pursuer 2

    if fa2==fmin
       ke2 = fmax+fmidd-2*pi-2*fmin;

    elseif fa2==fmidd
       ke2 = fmax+fmin-2*fmidd;

    else 
       ke2 = 2*pi+fmin+fmidd-2*fmax;
    end
    

    m2 = -abs(ke2)/(4*pi-asin(r3.k)+asin(r1.k))*sin(pi*((r12)/(r11+r12+r13))^(log(2)/log(3)));
    beta2 = pi/2*(1-exp(m2));
    
    vrx2= -vi2*sin(beta2)*sign(ke2)*sin(fa2)-vi2*cos(beta2)*cos(fa2);
    vry2= vi2*sin(beta2)*sign(ke2)*cos(fa2)-vi2*cos(beta2)*sin(fa2);

    r2.x=r2.x+ vrx2;
    r2.y=r2.y+ vry2;
    
  % strategy of pursuer 3
 
     if fa3==fmin
       ke3 = fmax+fmidd-2*pi-2*fmin;

    elseif fa3==fmidd
       ke3 = fmax+fmin-2*fmidd;

    else 
       ke3 = 2*pi+fmin+fmidd-2*fmax;
    end


    m3 = -abs(ke3)/(4*pi-asin(r1.k)+asin(r2.k))*sin(pi*((r13)/(r11+r12+r13))^(log(2)/log(3)));
    beta3 = pi/2*(1-exp(m3));
    
    vrx3= -vi3*sin(beta3)*sign(ke3)*sin(fa3)-vi3*cos(beta3)*cos(fa3);
    vry3= vi3*sin(beta3)*sign(ke3)*cos(fa3)-vi3*cos(beta3)*sin(fa3);

    r3.x=r3.x+ vrx3;
    r3.y=r3.y+ vry3;
 
 
    
end 






















