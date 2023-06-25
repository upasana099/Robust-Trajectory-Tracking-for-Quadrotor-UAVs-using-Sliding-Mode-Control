function x = quinticpoly(t0,tf,q0,qf,qd0,qdf,qdd0,qddf)

A = [t0^5 t0^4 t0^3 t0^2 t0 1;
     5*t0^4 4*t0^3 3*t0^2 2*t0 1 0;
     20*t0^3 12*t0^2 6*t0 2 0 0;
     tf^5 tf^4 tf^3 tf^2 tf 1;
     5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
     20*tf^3 12*tf^2 6*tf 2 0 0];

b = [q0 qd0 qdd0 qf qdf qddf]';

x = A\b;
end
