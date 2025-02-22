%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/05/2022
% Control NMMPC-NMHE-Racing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = fullmodel(t, states, u)
 
    m = 1.98;
    I = 0.1217;
    lf= 0.164;
    lr= 0.16;
    C1 = lr/(lr+lf);%0.5;
    C2 = 1/(lr+lf);%17.06;
    Cr0 = 0.6;
    Cr2 = 0.1;
    Cm1 = 12.0;
    Cm2 = 2.17;
    Bf = 29.495;
    Br = 26.9705;
    Cf = 0.0867;
    Cr = 0.1632;
    Df = 42.5255;
    Dr = 161.585;
    %%%%%%%%%%%%%%%%%%%%
    
    % States
    x      = states(1);
    y      = states(2);
    phi    = states(3);
    vx     = states(4);      
    vy     = states(5);     
    w      = states(6);      

    % Inputs:
    delta   = u(1);
    a       = u(2); 
    

    if abs(vx) > 0.1
        % Front and rear slip angles:
        af = delta - atan((lf*w + vy)/vx);
        ar = atan((lr*w - vy)/vx);
    else
        af = 0.0;
        ar = 0.0;    
    end
    
%     af = delta - atan((w*lf + vy)/(vx+0.0001)) ;
%     ar = atan((w*lr - vy)/(vx+ 0.0001));

    Fxr = (Cm1-Cm2*vx)*a -Cr2*vx^2-Cr0;
    Fyf = Df*sin(Cf*atan(Bf*af));
    Fyr = Dr*sin(Cr*atan(Br*ar));

    dx(1,1) =  vx*cos(phi) - vy*sin(phi);               %[x]    
    dx(2,1) =  vx*sin(phi) + vy*cos(phi);              %[y]
    dx(3,1) =  w;                                     %[phi]
    dx(4,1) = (Fxr - Fyf*sin(delta))/m + w*vy;         %[vx] 
    dx(5,1) = (Fyf*cos(delta) + Fyr)/m - w*vx;          %[vy]
    dx(6,1) = (Fyf*lf*cos(delta) - Fyr*lr)/I;           %[w]
    
    
end