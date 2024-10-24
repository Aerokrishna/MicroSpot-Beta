import math

def inverse_kinematics(x ,z):
    a = 0.107
    b = 0.130
    
    l = math.sqrt(x**2 + z**2)
    phi = math.acos(( a**2 + l**2 - b**2)/(2 * a * l))

    #shoulder angle
    theta1 = phi + math.atan(z/x) - ((math.pi)/2)

    #knee angle
    theta2 = math.pi - math.acos(( a**2 + b**2 - l**2)/(2 * a * b))
    if theta1<0:
        theta1 = theta1 + math.pi
        
    theta = [theta1,theta2]
    return theta

