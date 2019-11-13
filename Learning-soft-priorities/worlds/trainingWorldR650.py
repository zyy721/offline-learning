import pydart2 as pydart
import numpy as np
from controllers import manipulatorController

class trainingWorldR650(pydart.World):
    def __init__(self, step, setGravity=False):
        # pydart.World.__init__(self, 0.001)
        pydart.World.__init__(self, step, "./data/skel/two_cubes.skel")
        self.setGravity = setGravity
        if self.setGravity is True:
            self.set_gravity([0.0, -9.81, 0.0])
            print('Gravity Is On')
        else:
            self.set_gravity([0.0, 0.0, 0.0])
            print('Gravity Is Off')
        # print('pydart create_world OK')

        self.robot = self.add_skeleton("./data/KR5/KR5 sixx R650.urdf")
        # print('pydart add_skeleton OK')
        self.JointAngle = np.zeros((6, 0))
        self.JointVelocity = np.zeros((6, 0))
        self.JointAcceleration = np.zeros((6, 0))
        self.JointTau = np.zeros((6, 0))

        self.energy = 0.0
        self.goalError = 0.0
        self.fitness = 0
        self.minDist = 10
        self.collision = False


    def step(self, ):
        super(trainingWorldR650, self).step()

        tau = (self.robot.tau).reshape((self.robot.ndofs, 1))
        q = (self.robot.q).reshape((self.robot.ndofs, 1))
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))
        ddq = (self.robot.ddq).reshape((self.robot.ndofs, 1))
        self.JointTau = np.column_stack((self.JointTau, tau))
        self.JointAngle = np.column_stack((self.JointAngle, q))
        self.JointVelocity = np.column_stack((self.JointVelocity, dq))
        self.JointAcceleration = np.column_stack((self.JointAcceleration, ddq))




        num_contacts = self.collision_result.num_contacts()
        if num_contacts != 0:
            self.collision = True

        u = np.array(self.robot.accelerations())
        self.energy += np.linalg.norm(u)
        # print(self.energy)

        transform = self.robot.bodynodes[6].world_transform()
        eef = transform[[0, 1, 2], 3]

        self.error = np.linalg.norm(eef - np.array([0.5, 0.16, 0.0]))
        self.goalError += self.error

        md = np.linalg.norm(eef - np.array([0.4, 0.41, eef[2]]))
        if md < self.minDist:
            self.minDist = md

    def getFitnessValue(self):
        return self.fitness

    def getResult(self):
        #seek maximum
        self.fitness= -(self.energy/20 + self.goalError/100)/2.0
        FitnessValue = self.fitness

        MinDist = self.minDist*1000 # unit: millimeter

        Collision = self.collision

        '''if Collision is True:
            FitnessValue = 0
            MinDist = 0
            print('Collision Happened During Training')'''

        print('Energy Term:: ')
        print(self.energy/20)
        print('Error Term:: ')
        print(self.goalError/100)
        print('Collision: ')
        print(Collision)
        print('Min dist:')
        print(MinDist)
        # MinDist += 200
        print('Final Error: ')
        print(self.error)
        print('FitnessValue: ')
        print(FitnessValue)
        return [FitnessValue, MinDist, Collision]

    def draw_with_ri(self, ri):
        ri.set_color(0, 0, 0)
        ri.draw_text([20, 40], "time = %.4fs" % self.t)
        transform = self.robot.bodynodes[6].world_transform()
        self.error = np.linalg.norm(transform[[0, 1, 2], 3] - np.array([0.5, 0.16, 0.0]))
        ri.draw_text([20, 60], "end-effector error = %.4fm" % self.error)





