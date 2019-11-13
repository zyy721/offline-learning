# Copyright 2018-2019 AIRS
#
# \author Jian Li
#
import numpy as np

class positionTask:
    """!@brief
    It generates joint acceleration given a desired position.
    """

    def __init__(self, skel, setPoint, taskWeight=None, selectionVector=None, Kd=None, Kp=None, bodyNodeIndex=None):

        self.robot = skel

        if Kd is None:
            self.Kd = 10
        else:
            self.Kd = Kd

        if Kp is None:
            self.Kp = 2
        else:
            self.Kp = Kp

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if setPoint is None:
            raise Exception("Desired angle is not set")
        else:
            self.desiredPosition = setPoint

        if taskWeight is None:
            self.taskWeight = 1.0
        else:
            self.taskWeight = taskWeight

        if selectionVector is None:
            self.selectionMatrix = np.identity(setPoint.shape[0])
        else:
            self.selectionMatrix = np.identity(setPoint.shape[0])
            for i in range(setPoint.shape[0]):
                self.selectionMatrix[i, i] = selectionVector[i]

        self.error = np.zeros((setPoint.shape[0], 1))

    def cal_QP_Matricies(self):
        newJacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian = self.selectionMatrix.dot(newJacobian)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        translation = transform[[0, 1, 2], 3].reshape((3, 1))

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.error = self.selectionMatrix.dot(translation - self.desiredPosition)

        constant = (newJacobian_dot + self.Kd * newJacobian).dot(dq) + self.Kp * (self.error)

        Q = newJacobian.T.dot(newJacobian)

        P = 2 * constant.T.dot(newJacobian)

        C = constant.T.dot(constant)

        return [self.taskWeight*Q, self.taskWeight*P, self.taskWeight*C]

    def cal_LS_Matricies(self):

        return 0