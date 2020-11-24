pkg load symbolic

syms w x y z

t = [3.5*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y))+3*(sin(w)*cos(x)*sin(z)+cos(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y))),
     3.5*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))+3*(cos(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))-cos(w)*cos(x)*sin(z)),
     3.5*cos(x)*cos(y)+3*(cos(x)*cos(y)*cos(z)-sin(x)*sin(z))+2.5];

jac = jacobian(t, [w, x, y, z]);
disp(jac)
