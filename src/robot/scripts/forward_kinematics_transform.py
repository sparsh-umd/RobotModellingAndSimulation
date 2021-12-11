from sympy import *

theta, alpha, d, a = symbols('theta alpha d a')
d1, d2, d3, d4 = symbols('d1 d2 d3 d4')
theta1, theta2, theta3 = symbols('theta1, theta2, theta3')

A = Matrix([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)], 
    [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)], 
    [0, sin(alpha), cos(alpha), d], 
    [0, 0, 0, 1]])

links = [
    [0, d1, -pi/2, theta1 - pi/2],
    [d2, 0, 0, theta2 - pi/2],
    [d3, 0, -pi/2, theta3],
    [0, -d4, 0, 0]
]

Ais = []
for i in range(len(links)):
    Ai = A.subs({a: links[i][0], d: links[i][1], alpha: links[i][2], theta: links[i][3]})
    Ais.append(Ai)

transformation_mats = []
cur_t_mat = -1
for i in range(len(Ais)):
    if cur_t_mat < 0:
        transformation_mats.append(Ais[i])
        cur_t_mat += 1
    else:
        new_t_mat = transformation_mats[cur_t_mat] * Ais[i]
        transformation_mats.append(new_t_mat)
        cur_t_mat += 1

final_mat = transformation_mats[len(transformation_mats) - 1]
print('The transformation matrix.\n')
pprint(final_mat)
print('\n--------------------------------------\n')

s_final_mat = final_mat.subs({theta1: 0, theta2: 0, theta3: 0})
print('The transformation matrix after substituting all theta values as 0.\n')
pprint(s_final_mat)
print('\n--------------------------------------\n')

s_final_mat = final_mat.subs({theta1: pi/2, theta2: 0, theta3: 0})
print('The transformation matrix after substituting theta1 as 90 degrees and all other theta values as 0.\n')
pprint(s_final_mat)
print('\n--------------------------------------\n')

s_final_mat = final_mat.subs({theta1: 0, theta2: pi/2, theta3: 0})
print('The transformation matrix after substituting theta2 as 90 degrees and all other theta values as 0.\n')
pprint(s_final_mat)
print('\n--------------------------------------\n')