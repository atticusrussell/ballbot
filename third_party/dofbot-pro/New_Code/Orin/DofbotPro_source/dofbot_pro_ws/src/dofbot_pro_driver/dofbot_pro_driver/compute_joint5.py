import math
def compute_joint5(vy,vx):           
    angle_radians = math.atan2(vy, vx)
    angle_degrees = math.degrees(angle_radians)
    return angle_degrees