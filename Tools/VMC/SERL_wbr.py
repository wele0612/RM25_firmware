import math

# Physical Design Parameters
L1 = 210
L2 = 200
d1 = 94.5
d2 = 110
d3 = 70
d4 = 90

# angles, as input
theta1 = math.pi/3
theta2 = math.pi/4

#theta3 = math.pi/3

# Forward Kinetics

def forward_kinetics(theta1, theta2):
    theta12=theta1+theta2
    d5 = math.sqrt(d1**2 + d4**2 - 2*d1*d4*math.cos(theta12))
    # print("d5 = ", d5)

    theta3_p1 = math.acos((d4**2 + d5**2 - d1**2)/(2*d4*d5))
    theta3_p2 = math.acos((d3**2 + d5**2 - d2**2)/(2*d3*d5))
    theta3 = theta3_p1 + theta3_p2

    print(f"theta3 = {theta3} rad / {math.degrees(theta3)} deg")

    L = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(theta3))
    theta4 = math.asin(L1/L*math.sin(theta3))
    theta = theta1-theta4
    print("L = ", L)
    print(f"theta = {theta} rad / {math.degrees(theta)} deg")
    return (L, theta)

(L,theta) = forward_kinetics(theta1, theta2)
