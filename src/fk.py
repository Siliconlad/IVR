def fk(angles):
    from math import sin, cos
    w, x, y, z = angles
    t = np.array([[sin(w)*cos(x)*sin(z)+cos(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)), sin(w)*cos(x)*cos(z)-sin(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)), cos(w)*cos(y)-sin(w)*sin(x)*sin(y), 3.5*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y))+3*(sin(w)*cos(x)*sin(z)+cos(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)))],
                  [cos(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))-cos(w)*cos(x)*sin(z), -cos(w)*cos(x)*cos(z)-sin(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y)), cos(w)*sin(x)*sin(y)+sin(w)*cos(y), 3.5*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))+3*(cos(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))-cos(w)*cos(x)*sin(z))],
                  [cos(x)*cos(y)*cos(z)-sin(x)*sin(z), -cos(x)*cos(y)*sin(z)-sin(x)*cos(z), -cos(x)*sin(y), 3.5*cos(x)*cos(y)+3*(cos(x)*cos(y)*cos(z)-sin(x)*sin(z))+2.5],
                  [0, 0, 0, 1]])
    orientation = t[:3, :3]
    position = t[:3, -1]
    return position, orientation

a = [0, 0, 0, 0]
pos, ori = fk(a)
print(pos)
print(ori)
