% Mp = 0.027;
% lp = 0.153;
% r = 0.08260;
% Jm = 3.00e-005;
% g = 9.81;
% Jeq = 1.23e-004;
% Jp = 1.1e-004;
% Beq = 0;
% Bp = 0;
% Rm = 3.3;
% Kt = 0.2797;
% Km = 0.2797;

Mp = 0.0677;
lp = 0.12;
r = 0.0925;
Jm = 3.00e-005;
g = 9.81;
Jeq = 0.00012;
Jp = 0.00009398592;
Beq = 0;
Bp = 0.1;
Rm = 3.3;
Kt = 0.2797;
Km = 0.2797;

 A = [0 0 1 0;
      0 0 0 1;
      0 (r*Mp^2*lp^2*g)/(Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2) (-Kt*Km*(Jp+Mp*lp^2))/((Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2)*Rm)*0.01 0;
      0 (Mp*lp*g*(Jeq+Mp*r^2))/(Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2) (-Mp*lp*Kt*r*Km)/((Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2)*Rm)*0.01 0]
 
 B = [0;
      0;
      (Kt*(Jp+Mp*lp^2))/((Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2)*Rm)*0.1;
      (Mp*lp*Kt*r)/((Jp*Jeq+Mp*lp^2*Jeq+Jp*Mp*r^2)*Rm)*0.1; ]
 
  %     rotor arm      pendulum
  Q = [1, 0, 0, 0; 0, 15, 0, 0; 0, 0, 4, 0; 0, 0, 0, 2.5; ];
  R = 0.00005;
  K = lqr(A,B,Q,R);
  
  disp(K);
  
tspan = 0:.00001:1;
y0 = [1; 0; pi-0.5; 0];
y1 = -[1; 0; pi; 0];
z = (A-B*K);
[t,y] = ode45(@(t,y)((A-B*K)*(y+y1)),tspan,y0);

figure
plot(t,y);