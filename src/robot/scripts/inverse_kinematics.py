from typing import List
from sympy import *
import matplotlib.pyplot as plt

class Kinematics:
    '''
    To plot the circle drawn by the end effector just use the plot_end_eff function or just run this file.
    plot_end_eff function first performs inverse kinematics to generate joint angles and joint velocities at each
    discrete point in the circle and then performs forward kinematics to generate the end effector coordinates.

    NOTE: The x coordinates of the final circle obtained are slightly shifted because I had to give a very small
    angle to the joint 6 to make the Jacobian invertible at the starting position of the end effector. If I did not
    have to do this the circle would be perfect.
    '''
    def __init__(self, total_time: int, time_step: float) -> None:
        # Transformation matrices and J
        self.theta, self.alpha, self.d, self.a = symbols('theta alpha d a')
        self.d1, self.d2, self.d3, self.d4 = symbols('d1 d2 d3 d4')
        self.theta1, self.theta2, self.theta3 = symbols('theta1 theta2 theta3')
        self.A = Matrix([[cos(self.theta), -sin(self.theta) * cos(self.alpha), sin(self.theta) * sin(self.alpha), self.a * cos(self.theta)], 
            [sin(self.theta), cos(self.theta) * cos(self.alpha), -cos(self.theta) * sin(self.alpha), self.a * sin(self.theta)], 
            [0, sin(self.alpha), cos(self.alpha), self.d], 
            [0, 0, 0, 1]])
        self.links = [
            [0, self.d1, -pi/2, self.theta1 - pi/2],
            [self.d2, 0, 0, self.theta2 - pi/2],
            [self.d3, 0, -pi/2, self.theta3],
            [0, -self.d4, 0, 0]
        ]
        self.transformation_mats = []
        self.J = None

        # Points on circle
        self.p_thetas = []
        self.total_time = total_time
        self.time_step = time_step
        self.theta_dot = (2 * pi) / self.total_time
        self.theta_step = self.theta_dot * self.time_step
        self.x, self.y, self.z, self.p_theta, self.x_dot, self.y_dot, self.z_dot = symbols('x y z p_theta x_dot y_dot z_dot')
        self.x = -0.57 * sin(self.p_theta)
        self.y = -0.57 * cos(self.p_theta)
        self.z = 0.24
        self.x_dot = -0.57 * cos(self.p_theta) * (self.theta_dot)
        self.y_dot = 0.57 * sin(self.p_theta) * (self.theta_dot)
        self.z_dot = 0
        self.points_coordinates = []
        self.point_derivatives = []

        # forward and inverse kinematics variables
        self.joint_angles = []
        self.joint_velocities = []
        self.end_eff_coordinates = []
    
    def gen_t_mats(self) -> None:
        Ais = []
        for i in range(len(self.links)):
            Ai = self.A.subs({self.a: self.links[i][0], self.d: self.links[i][1],
                        self.alpha: self.links[i][2], self.theta: self.links[i][3]})
            Ais.append(Ai)

        self.transformation_mats = []
        cur_t_mat = -1
        for i in range(len(Ais)):
            if cur_t_mat < 0:
                self.transformation_mats.append(Ais[i])
                cur_t_mat += 1
            else:
                new_t_mat = self.transformation_mats[cur_t_mat] * Ais[i]
                self.transformation_mats.append(new_t_mat)
                cur_t_mat += 1
    
    def gen_J(self) -> None:
        if len(self.transformation_mats) == 0:
            self.gen_t_mats()
        Zs = []
        Os = []
        Zs.append(Matrix([[0], [0], [1]]))
        Os.append(Matrix([[0], [0], [0]]))
        for i in range(len(self.transformation_mats)):
            Zs.append(self.transformation_mats[i][0:3, 2])
            Os.append(self.transformation_mats[i][0:3, 3])

        On = Os[len(Os) - 1]
        j1 = Zs[0].cross(On - Os[0])
        j2 = Zs[1].cross(On - Os[1])
        j3 = Zs[2].cross(On - Os[2])
        j4 = Zs[3].cross(On - Os[3])

        J1 = j1.col_join(Zs[0])
        J2 = j2.col_join(Zs[1])
        J3 = j3.col_join(Zs[2])
        J4 = j4.col_join(Zs[3])

        J = J1.row_join(J2).row_join(J3).row_join(J4)
        self.J = J.subs({self.d1: 0.205, self.d2: 0.35, self.d3: 0.22, self.d4: 0.035})

        print('The Jacobian matrix.\n')
        for row in range(6):
            for col in range(4):
                print('The element (' + str(row + 1) + ', ' + str(col + 1) + ') is: \n')
                pprint(J[row, col])
                print('\n--------------------------------------\n')

    def evaluate_J(self, angles: List[Float]) -> Matrix:
        if len(angles) != 4:
            raise ValueError('evaluate_J should be called with a list of 4 angles!')
        if not self.J:
            self.gen_J()
        return N(self.J.subs({self.theta1: angles[0], 
            self.theta2: angles[1], 
            self.theta3: angles[2]}))
    
    def evaluate_J_inv(self, angles: List[Float]) -> Matrix:
        return self.evaluate_J(angles).pinv()
    
    def gen_p_thetas(self) -> None:
        self.p_thetas = []
        theta = 0
        while theta <= 2 * pi + self.theta_step:
            self.p_thetas.append(N(theta))
            theta += self.theta_step
    
    def gen_point_coordinates(self) -> None:
        if len(self.p_thetas) == 0:
            self.gen_p_thetas()
        self.points_coordinates = []
        for p_theta in self.p_thetas:
            self.points_coordinates.append([N(self.x.subs({self.p_theta: p_theta})), N(self.y.subs({self.p_theta: p_theta})), self.z])
    
    def gen_point_derivatives(self) -> None:
        if len(self.p_thetas) == 0:
            self.gen_p_thetas()
        self.point_derivatives = []
        for p_theta in self.p_thetas:
            self.point_derivatives.append([N(self.x_dot.subs({self.p_theta: p_theta})), N(self.y_dot.subs({self.p_theta: p_theta})), self.z_dot])
    
    def setup(self) -> None:
        self.gen_J()
        self.gen_point_coordinates()
        self.gen_point_derivatives()

    def to_column_vector_4(self, list: List[float]) -> Matrix:
        if len(list) > 4:
            raise ValueError('Cannot convert a list of length more than four to column_vector')
        ans = Matrix([[0], [0], [0], [0]])
        for i in range(len(list)):
            ans[i, 0] = list[i]
        return ans
    
    def to_column_vector_6(self, list: List[float]) -> Matrix:
        if len(list) > 6:
            raise ValueError('Cannot convert a list of length more than six to column_vector')
        ans = Matrix([[0], [0], [0], [0], [0], [0]])
        for i in range(len(list)):
            ans[i, 0] = list[i]
        return ans
    
    def to_list_4(self, mat: Matrix) -> List[Float]:
        if mat.shape[0] != 4 and mat.shape[1] != 1:
            raise ValueError('Matrix needs to be (4,1) to convert to list!')
        ans = []
        for row in range(mat.shape[0]):
            ans.append(mat[row, 0])
        return ans
    
    def to_list_3(self, mat: Matrix) -> List[Float]:
        if mat.shape[0] != 3 and mat.shape[1] != 1:
            raise ValueError('Matrix needs to be (3,1) to convert to list!')
        ans = []
        for row in range(mat.shape[0]):
            ans.append(mat[row, 0])
        return ans
    
    def inverse_kinematics(self) -> None:
        self.setup()
        self.joint_angles = [[0, pi/2, 0, 0]]
        self.joint_velocities = []
        
        for i in range(len(self.p_thetas)):
            point_derivative = self.point_derivatives[i]
            joint_angle = self.joint_angles[i]
            J_inv = self.evaluate_J_inv(joint_angle)
            joint_velocity = self.to_list_4(J_inv * self.to_column_vector_6(point_derivative))
            self.joint_velocities.append(joint_velocity)

            if i != len(self.p_thetas) - 1: 
                next_joint_angle = [a + (self.time_step * b) for (a, b) in zip(joint_angle, joint_velocity)]
                self.joint_angles.append(next_joint_angle)

    def forward_kinematics(self) -> None:
        self.inverse_kinematics()
        self.end_eff_coordinates = [[0, -0.57, 0.24]]
        
        for i in range(len(self.p_thetas)):
            end_eff_coordinate = self.end_eff_coordinates[i]
            joint_angle = self.joint_angles[i]
            joint_velocity = self.joint_velocities[i]
            J = self.evaluate_J(joint_angle)
            end_eff_velocity = self.to_list_3(J * self.to_column_vector_4(joint_velocity))

            if i != len(self.p_thetas) - 1:
                next_end_eff_coordinates = [a + (self.time_step * b) for (a, b) in zip(end_eff_coordinate, end_eff_velocity)]
                self.end_eff_coordinates.append(next_end_eff_coordinates)

    def plot_end_eff(self) -> None:
        self.forward_kinematics()
        x = []
        y = []
        for end_eff_coordinate in self.end_eff_coordinates:
            x.append(end_eff_coordinate[0])
            y.append(end_eff_coordinate[1])
        plt.plot(x, y)
        plt.show()

if __name__ == '__main__':
    k = Kinematics(5, 0.1)
    k.plot_end_eff()
 