import pydart2 as pydart
import numpy as np
from controllers import manipulatorController

class r650(pydart.World):
    def __init__(self, ):
        # pydart.World.__init__(self, 0.001)
        pydart.World.__init__(self, 0.001, "./data/skel/two_cubes.skel")

        self.set_gravity([0.0, -9.81, 0.0])

        # self.set_gravity([0.0, -9.81, 0.0])

        print('pydart create_world OK')

        self.robot = self.add_skeleton("./data/KR5/KR5 sixx R650.urdf")

        print('pydart add_skeleton OK')

        # Initialize the controller
        # self.controller = controller.Controller(self.robot)
        # self.robot.set_controller(self.controller)
        print('create controller OK')


    '''def step(self, ):
        super(r650, self).step()
        print(self.collision_result.num_contacts())'''