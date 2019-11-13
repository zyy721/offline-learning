from scipy import *
from scipy.linalg import norm, pinv
from matplotlib import pyplot as plt
import numpy as np

class RBFN:
     
    def __init__(self, indim, numCenters, outdim, time, centers=None):
        self.indim = indim
        self.outdim = outdim
        self.numCenters = numCenters
        #not sure if it is right
        if centers is None:
            self.centers = np.linspace(0, time, numCenters)
        else:
            self.centers = centers
        # self.centers = [random.uniform(-1, 1, indim) for i in range(numCenters)]
        self.beta = 0.05
        self.W = random.random((self.numCenters, self.outdim))

    def _basisfunc(self, c, d):
        return exp(-self.beta * norm(c-d)**2)
     
    def _calcAct(self, X):
        # calculate activations of RBFs
        G = zeros((X.shape[0], self.numCenters), float)
        for ci, c in enumerate(self.centers):
            for xi, x in enumerate(X):
                G[xi,ci] = self._basisfunc(c, x)
        return G
    
    def train(self, X, Y):
        # choose random center vectors from training set
        '''rnd_idx = random.permutation(X.shape[0])[:self.numCenters]
        self.centers = [X[i,:] for i in rnd_idx]
        print ("center", self.centers)'''
        # calculate activations of RBFs
        G = self._calcAct(X)

        # calculate output weights (pseudoinverse)
        self.W = dot(pinv(G), Y)
        print(self.W)
    
    def test(self, X):
        """ X: matrix of dimensions n x indim """
        G = self._calcAct(X)
        Y = dot(G, self.W)
        return Y
    
    def setHyperParams(self, W):
        assert W.shape[0] == self.numCenters
        assert W.shape[1] == self.outdim
        self.W = W

    def _sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def calOutput(self, X):
        """ X: 1 x indim """
        #calculate activation value
        G = self._calcAct(X)
        #sum up and normalize
        Y = dot(G, self.W)/sum(G)
        Y = self._sigmoid(Y)
        return Y
 
      
if __name__ == '__main__':
    
    time = 10
    rbfn = RBFN(1, 5, 1, time)
    # W = random.random((rbfn.numCenters, rbfn.outdim))



    # rbfn.setHyperParams(W)
    # print(W)

    x = np.arange(0, time, 0.005)
    # print(len(x))
    # print(x)
    y = np.sin(x)*0.05 # - np.sin(x)*0.05
    plt.plot(x, y, 'k-')
    # print(len(y))
    # print(y)
    rbfn.train(x, y)

    y = zeros((0,1))
    for point in x:
        point = np.array([point])
        # print(y.shape)
        # print((np.array(rbfn.calOutput(point))).shape)
        y = np.row_stack((y, np.array(rbfn.calOutput(point))))

    plt.figure(figsize=(12, 8))
    # print(y.shape)
    y = np.transpose([y])
    # print(y.shape)
    plt.plot(x, y[0], 'k-')
    # plt.plot(x, y[1], 'k-')
    # plt.plot(x, y[2], 'k-')
    plt.show()
