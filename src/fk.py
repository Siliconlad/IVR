from math import sin, cos

def fk(angles):
    w, x, y, z = angles
    # Forward Kinematics Matrix (4x4), contains orientation and position
    t = np.array([[sin(w)*cos(x)*sin(z)+cos(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)),  sin(w)*cos(x)*cos(z)-sin(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)), cos(w)*cos(y)-sin(w)*sin(x)*sin(y), 3.5*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y))+3*(sin(w)*cos(x)*sin(z)+cos(z)*(sin(w)*sin(x)*cos(y)+cos(w)*sin(y)))],
                  [cos(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))-cos(w)*cos(x)*sin(z), -cos(w)*cos(x)*cos(z)-sin(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y)), cos(w)*sin(x)*sin(y)+sin(w)*cos(y), 3.5*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))+3*(cos(z)*(sin(w)*sin(y)-cos(w)*sin(x)*cos(y))-cos(w)*cos(x)*sin(z))],
                  [                              cos(x)*cos(y)*cos(z)-sin(x)*sin(z),                               -cos(x)*cos(y)*sin(z)-sin(x)*cos(z),                     -cos(x)*sin(y),                                                  3.5*cos(x)*cos(y)+3*(cos(x)*cos(y)*cos(z)-sin(x)*sin(z))+2.5],
                  [                                                               0,                                                                 0,                                  0,                                                                                                             1]])
    orientation = t[:3, :3]
    position = t[:3, -1]
    return position, orientation

def jacobian(angles):
    # Jacobian Matrix (3x4)
    j = np.array([[(7*cos(w)*cos(y)*sin(x))/2 - 3*cos(z)*(sin(w)*sin(y) - cos(w)*cos(y)*sin(x)) - (7*sin(w)*sin(y))/2 + 3*cos(w)*cos(x)*sin(z), (7*cos(x)*cos(y)*sin(w))/2 - 3*sin(w)*sin(x)*sin(z) + 3*cos(x)*cos(y)*cos(z)*sin(w), (7*cos(w)*cos(y))/2 + 3*cos(z)*(cos(w)*cos(y) - sin(w)*sin(x)*sin(y)) - (7*sin(w)*sin(x)*sin(y))/2,   3*cos(x)*cos(z)*sin(w) - 3*sin(z)*(cos(w)*sin(y) + cos(y)*sin(w)*sin(x))],
                  [(7*cos(w)*sin(y))/2 + 3*cos(z)*(cos(w)*sin(y) + cos(y)*sin(w)*sin(x)) + (7*cos(y)*sin(w)*sin(x))/2 + 3*cos(x)*sin(w)*sin(z), 3*cos(w)*sin(x)*sin(z) - (7*cos(w)*cos(x)*cos(y))/2 - 3*cos(w)*cos(x)*cos(y)*cos(z), (7*cos(y)*sin(w))/2 + 3*cos(z)*(cos(y)*sin(w) + cos(w)*sin(x)*sin(y)) + (7*cos(w)*sin(x)*sin(y))/2, - 3*sin(z)*(sin(w)*sin(y) - cos(w)*cos(y)*sin(x)) - 3*cos(w)*cos(x)*cos(z)],
                  [                                                                                                                          0,                    - (7*cos(y)*sin(x))/2 - 3*cos(x)*sin(z) - 3*cos(y)*cos(z)*sin(x),                                                     - (7*cos(x)*sin(y))/2 - 3*cos(x)*cos(z)*sin(y),                                 - 3*cos(z)*sin(x) - 3*cos(x)*cos(y)*sin(z)]])
     


a = [0, 0, 0, 0]
pos, ori = fk(a)
print(pos)
print(ori)
