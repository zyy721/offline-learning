import pydart2 as pydart
import numpy as np
from scipy import *
from utils import print_skeleton
from worlds import trainingWorldR650
from controllers import manipulatorController

class evaluationInter:

    def __init__(self, runtime, hyperParam=None):

        pydart.init()
        #print('pydart initialization OK')

        self.runtime = runtime

        self.world = trainingWorldR650.trainingWorldR650(step=0.1, setGravity=False)
        #print('pydart create_world OK')

        self.skel = self.world.skeletons[-1] #print(world.skeletons[-1]): [Skeleton(2): KR5sixxR650WP_description]

        self.hyperParams = random.random((5, 3))

        self.fitness = -1
        # print_skeleton.skeleton_printer(skel)

    def updateHyperParams(self, hyperParams=None):
        if hyperParams.all()==None:
            print("Error: HyperParams is None")
        else:
            self.hyperParams = hyperParams.reshape((5,3))
            print('Update hyperParams OK')

        self.world.controller = manipulatorController.manipulatorController(self.skel,
                                self.hyperParams, jointAngle=ones((6, 1)), runtime=self.runtime, setGravity=self.world.setGravity)

        self.skel.set_controller(self.world.controller)
        # print('Create controller OK')

    def evaluation(self):
        '''initiate'''
        # self.skel.q = array([1.5, 1.5, -0.5, 0.0, 0.0, 0.0])
        self.skel.q = array([1.5, 1.0, -0.5, 0.0, 0.0, 0.0])

        while self.world.t <= self.runtime:
            self.world.step()

        # fitness = self.world.getFitnessValue()
        result = self.world.getResult()
        return result

    def gui(self):
        self.skel.q = array([1.5, 1.0, -0.5, 0.0, 0.0, 0.0])
        win = pydart.gui.viewer.launch(self.world, "Safe Learning Priorities", default_camera=0)  # Use Z-up camera
        win.run_application()

    def guiStatic(self):
        # self.skel.q = array([1.5, 1.0, -0.5, 0.0, 0.0, 0.0])
        win = pydart.gui.viewer.launch(self.world, "Safe Learning Priorities", default_camera=0)  # Use Z-up camera
        win.run_application()

    def getJointTauCurve(self):
        JointTau = self.world.JointTau
        return JointTau

    def getJointAngleCurve(self):
        JointAngle = self.world.JointAngle
        return JointAngle

    def getJointVelocityCurve(self):
        JointVelocity = self.world.JointVelocity
        return JointVelocity

    def getJointAccelerationCurve(self):
        JointAcceleration = self.world.JointAcceleration
        return JointAcceleration


