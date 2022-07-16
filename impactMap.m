function xplus = impactMap(xminus)
global p;

q1 = xminus(1); 
q2 = xminus(2);

% Angular Momentum Impact Mapping
p11_plus = p.m*p.l*(p.l-p.b*cos(q1-q2))+p.m*p.a^2+p.mH*p.l^2;
p12_plus = p.m*p.b*(p.b-p.l*cos(q1-q2));
p21_plus = -p.m*p.b*p.l*cos(q1-q2);
p22_plus = p.m*p.b^2;
p11_minus = -p.m*p.a*p.b + (p.mH*p.l^2+2*p.m*p.a*p.l)*cos(q1-q2);
p12_minus = -p.m*p.a*p.b;
p21_minus = -p.m*p.a*p.b;
p22_minus = 0;
      
P_minus = [p11_minus,p12_minus;
          p21_minus,p22_minus];
 
P_plus = [p11_plus,p12_plus;
         p21_plus,p22_plus];
      
dtheta_minus = [xminus(3);xminus(4)];
dq_plus = (P_plus\P_minus)*dtheta_minus;

% Switch feet
R = [0,1;1,0];
q_plus = R*[q1;q2];
xplus = [q_plus; dq_plus];
end

