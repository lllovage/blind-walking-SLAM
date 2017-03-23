
cx(1).time = 0;                     cy(1).time = 0;
cx(1).type = 'p';                   cy(1).type = 'p';
cx(1).value = 0;                    cy(1).value = 0;
cx(1).Ts = 0.01;                    cy(1).Ts = 0.01;
%...................................................................
cx(2).time = tVia;                  cy(2).time = tVia;
cx(2).type = 'p';                   cy(2).type = 'p';
cx(2).value = r-r*cos(pi/6);        cy(2).value = r*sin(pi/6);
%...................................................................
cx(3).time = 2*tVia;                cy(3).time = 2*tVia;
cx(3).type = 'p';                   cy(3).type = 'p';
cx(3).value = r-r*cos(2*pi/6);      cy(3).value = r*sin(2*pi/6);
%..................................................................
cx(4).time = 3*tVia;                cy(4).time = 3*tVia;
cx(4).type = 'p';                   cy(4).type = 'p';
cx(4).value = r;                    cy(4).value = r;   
%..................................................................
cx(2).time = 4*tVia;                cy(2).time = 4*tVia;
cx(2).type = 'p';                   cy(2).type = 'p';
cx(2).value = 2*r-r*cos(2*pi/6);    cy(2).value = r*sin(2*pi/6);
%..................................................................
cx(2).time = 5*tVia;                cy(2).time = 5*tVia;
cx(2).type = 'p';                   cy(2).type = 'p';
cx(2).value = 2*r-r*cos(pi/6);      cy(2).value = r*sin(pi/6);
%..................................................................
cx(2).time = 6*tVia;                cy(2).time = 6*tVia;
cx(2).type = 'p';                   cy(2).type = 'p';
cx(2).value = 2*r;                  cy(2).value = 0;

xOut = genTraj (cx);                
yOut = genTraj (cy);