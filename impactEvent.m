function [value, isterminal, direction] = impactEvent(t,x)
    global p;
    q1     = x(1);
    q2     = x(2);
    dq1    = x(3);
    dq2    = x(4);
    y = p.l*(cos(q1 + p.psi) - cos(q2 + p.psi));
    ydot = p.l*(dq1*-sin(q1+p.psi) + dq2*sin(q2+p.psi));
    
    direction = -1;
    
    value = y;
    if ydot < 0 && dq2 < 0 
        isterminal = 1;
    else
       isterminal = 0;
    end
end