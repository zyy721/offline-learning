import numpy as np
from scipy import *
from RBFN import RBFN
from cvxopt import solvers, matrix, spmatrix
from controllers import positionTask
from controllers import jointAngleTask
import logging
import time
import os
class manipulatorController(object):
    """ Add damping force to the skeleton """
    def __init__(self, skel, hyperParam, jointAngle, runtime, setGravity=False, rbfnCenters=None, numCenters=5):
        self.skel = skel
        self.g = self.skel.world.gravity()
        self.setGravity = setGravity
        self.runtime = runtime
        self.rbfn = RBFN.RBFN(1, numCenters, 3, self.runtime, centers=rbfnCenters) # indim, numCenters, outdim, time
        self.rbfn.setHyperParams(hyperParam)
        self.jointAngle = jointAngle
        self.positionTask = positionTask.positionTask(skel, array([0.5, 0.16, 0.0]).reshape((3, 1)), bodyNodeIndex=6) # array([0.5, 0.16, 0.0])
        self.jointAngelTask = jointAngleTask.jointAngleTask(skel, array([0.0, 1.54, -2.296, 0.0, -0.8, 0.0]).reshape((6, 1)), Kp=0.5, Kd=5) # array([0.0, 1.54, -2.296, 0.0, -0.0, 0.0])
        self.elbowTask = positionTask.positionTask(skel, array([0.3614, -0.0291, 0.0]).reshape((3, 1)), Kp=10, Kd=15, bodyNodeIndex=4) # array([0.3614, -0.0191, 0.0])
        # # array([0.3614, -0.3191, 0.0]) for CMA-ES
    def jointPositionControl(self, jointAngle):

        dq = np.reshape(self.skel.dq, (self.skel.ndofs, 1))
        q = np.reshape(self.skel.q, (self.skel.ndofs, 1))
        error = jointAngle - q
        error_dt = -dq
        tau = 0.01*error + 0.001*error_dt
        tau = tau.flatten()
        print(self.skel.q)
        return tau

    def qpSolver(self, P, q, G=None, h=None, A=None, b=None, initvals=None):

        if G is None and h is None:
            args = [matrix(P), matrix(q)]
        else:
            args = [matrix(P), matrix(q), matrix(G), matrix(h)]

        try:
            sol = solvers.qp(*args)
        except ValueError:
            print("QP is infeasible")
            return -1

        if 'optimal' not in sol['status']:
            print("QP fails, the status are: %s", sol)
            return -1
        # print(sol['x'])
        jointAccl = np.array(sol['x']).reshape((q.shape[0], 1))

        return jointAccl

    def jointAccl2tau(self, jointAccl):

        tau = self.skel.M.dot(jointAccl)
        return tau

    def jointAccl2tauGravityCompensation(self, jointAccl):

        tau = self.skel.M.dot(jointAccl) + self.skel.coriolis_and_gravity_forces().reshape((self.skel.ndofs, 1))\
              - self.skel.constraint_forces().reshape((self.skel.ndofs, 1))
        return tau

    def compute(self):
        """!@brief
        Would be called automatically by "step()" function of the world object
        """
        time = np.array([self.skel.world.t])
        priorities = self.rbfn.calOutput(time).T
        # priorities = ones((3, 1))*0.5

        positionMatricies = self.positionTask.cal_QP_Matricies()
        jointAngleMatricies = self.jointAngelTask.cal_QP_Matricies()
        elbowMatricies = self.elbowTask.cal_QP_Matricies()

        Q = np.identity(self.skel.ndofs)

        positionArgs = [positionMatricies[0] + Q, positionMatricies[1].T]
        jointAngleArgs = [jointAngleMatricies[0] + Q, jointAngleMatricies[1].T]
        elbowArgs = [elbowMatricies[0] + Q, elbowMatricies[1].T]

        acc_lower = -0.3*(-1)
        positionArgs = [positionMatricies[0] + Q, positionMatricies[1].T, -1*np.identity(6), acc_lower*np.ones((6,1))]
        jointAngleArgs = [jointAngleMatricies[0] + Q, jointAngleMatricies[1].T, -1*np.identity(6), acc_lower*np.ones((6,1))]
        elbowArgs = [elbowMatricies[0] + Q, elbowMatricies[1].T, -1*np.identity(6), acc_lower*np.ones((6,1))]

        task_jointAccl = zeros((self.skel.ndofs, 0))
        task_jointAccl = np.column_stack((task_jointAccl, self.qpSolver(*positionArgs)))
        task_jointAccl = np.column_stack((task_jointAccl, self.qpSolver(*jointAngleArgs)))
        task_jointAccl = np.column_stack((task_jointAccl, self.qpSolver(*elbowArgs)))

        total_jointAccl = dot(task_jointAccl, priorities)

        '''single task'''
        #total_jointAccl = dot(task_jointAccl, priorities)  *0.333

        if self.setGravity is True:
            tau = self.jointAccl2tauGravityCompensation(total_jointAccl)
        else:
            tau = self.jointAccl2tau(total_jointAccl)

        tau = tau.flatten()

        # print(self.skel.bodynodes[4].world_transform())
        # print(self.skel.q)
        #print(self.skel.world.t)

        return tau