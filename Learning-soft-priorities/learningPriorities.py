from singleEpisode import evaluationInter

import GPy
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import safeopt
from RBFN import RBFN
from safeopt import linearly_spaced_combinations

runtime = 26
minFinalError = 100
optimalSolution = 0
optimalEvaluation = 0
inputDimension = 15

# evaluation function
def f(x):
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(x)
    result = evaluation.evaluation()
    y0 = np.ones((1,1))*result[0]
    # seek maximum
    return y0

def f_c(x):
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(x)
    result = evaluation.evaluation()
    return np.hstack([np.ones((1,1))*result[0], np.ones((1,1))*result[1]])

def cmaesf(x):
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(x)
    result = evaluation.evaluation()
    y0 = -np.ones((1,1))*result[0]
    # seek minimum
    return y0[0]

'''def bof(x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15):
    x = np.array([x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15])
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(x)
    result = evaluation.evaluation()
    y0 = np.ones((1,1))*result[0]
    # seek maximum
    return y0[0][0]'''

def bo_penalty_f(x):
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(x)
    result = evaluation.evaluation()
    if result[1] < 0:
        result[0] = result[0] + result[1]/500.0 # penalty item
    return result

def initiateSafeSet(inputDimension):
    y = np.ones((1,1))
    while True:
        x = (0.5-np.random.random((1, inputDimension)))*10
        evaluation = evaluationInter.evaluationInter(runtime=runtime)
        evaluation.updateHyperParams(x)
        result = evaluation.evaluation()
        y = result[0]

        if y < -0.7 and y > -0.8 and result[2] is False:
            print('initial x :')
            print(x)
            print('initial y :')
            print(y)
            return x
    return None

def cmaes_initiateSafeSet(inputDimension):
    while True:
        x = (0.5-np.random.random((1, inputDimension)))*10
        y = cmaesf(x)
        evaluation = evaluationInter.evaluationInter(runtime=runtime)
        evaluation.updateHyperParams(x)
        result = evaluation.evaluation()
        y = result[0]

        if y < -0.7 and y > -0.8 and result[2] is False:
            print('initial x :')
            print(x)
            print('initial y :')
            print(y)
            return x
    return x

if __name__ == '__main__':
    '''[ 3.98756312 -4.54692078 -4.20240821  3.37581306 -2.04208146  4.58879403
       0.60741451 -2.48077861 -0.49666369  0.76734985  0.04636265  1.39900566
      -2.89473381 -0.97787073 -2.90685335]
    '''
    # plot
    plt.close()
    plt.ion()

    # Measurement noise
    noise_var = 0.1 ** 2
    noise_var2 = 1e-2

    # Bounds on the inputs variable
    bounds = []
    for i in range(inputDimension):
        bounds.append((-10, 10))

    # Define Kernel
    # kernel = GPy.kern.RBF(input_dim=len(bounds), variance=2., lengthscale=1.0, ARD=True)
    kernel = GPy.kern.Matern52(input_dim=inputDimension, ARD=True)
    kernel2 = kernel.copy()

    # Initial safe point
    x0 = np.zeros((1, inputDimension))
    x0 = initiateSafeSet(inputDimension)
    '''x0 = np.array([-4.95957468, -3.61009445,  1.67003869, -4.88427162, -4.46011329, -1.38214976,
   2.15057211, -3.69813929,  4.4843571,   4.82027238, -0.72567684, -1.53831128,
   2.31816139,  2.08594099,  4.1214262]).reshape(1,15)'''

    # The statistical model of our objective function and safety constraint
    y0 = f(x0)
    xs = [0, 0]
    ys = [0, y0[0][0]]
    list_y = [y0[0][0]]
    print("initial x0 : ", x0)
    print("initial y0 : ", y0)

    # fitness value and constraints

    # gp = GPy.models.GPRegression(x0, y0, kernel, noise_var=noise_var)
    y0 = f_c(x0)
    gp = GPy.models.GPRegression(x0, y0[:, 0, None], kernel, noise_var=noise_var)
    gp2 = GPy.models.GPRegression(x0, y0[:, 1, None], kernel2, noise_var=noise_var2)

    # The optimization routine
    fmin = -10
    cmin = 0.1 #safety distance to obstacle
    # opt = safeopt.SafeOptSwarm(gp, fmin=fmin, bounds=bounds, threshold=0.2)

    while max(list_y) < -0.43:

        opt = safeopt.SafeOptSwarm([gp, gp2], [-np.inf, cmin], bounds=bounds, threshold=0.2)
        #parameter_set = linearly_spaced_combinations(bounds=bounds, num_samples = 1000)
        #opt = safeopt.SafeOpt([gp, gp2], parameter_set, [-np.inf, 0.], lipschitz=None, threshold=0.1)
        y0 = f(x0)
        xs = [0, 0]
        ys = [0, y0[0][0]]
        list_y = [y0[0][0]]
        plt.figure()
        plt.title('Fitness Value')
        fitnessData = [y0[0][0]]
        counter = [0]
        for i in range(250):
            # Obtain next query point
            x_next = opt.optimize() ######### ucb=False #########
            print("next x : ", x_next)

            # Get a measurement from the real system
            y_meas = f_c(x_next)
            print("evaluation : ", y_meas)

            # Add this to the GP model
            opt.add_new_data_point(x_next, y_meas)
            list_y.append(y_meas[0][0])
            print('order:')
            print(i)
            # plot
            if i % 5 is 0:
                xs[0] = xs[1]
                ys[0] = ys[1]
                xs[1] = i+5
                counter.append(i)
                ys[1] = max(list_y)
                fitnessData.append(ys[1])

                plt.plot(xs, ys, color='y')
            plt.pause(0.1)


    txtData = np.column_stack((counter, fitnessData))
    np.savetxt("fitnessValue.txt", txtData)

    '''Optimal Y:  [-0.48156585]
    Optimal X:  [-5.45050531  0.34979447  4.12126481 -0.43104242 -5.04246921  1.50982994
    2.09085143  2.27027575  0.82268795  3.28250759 -2.9581665  -0.07979856
    1.18092941 -3.61545808 -0.77355514]'''

    '''Optimal Y:  [-0.40701588]
    Optimal X:  [ 7.69542589  0.85522206 -1.20877597 -0.187846   -2.39027029  6.35187958
    4.5719057  -2.84564475  2.00812702 -2.17746402 -2.06790978  0.50882218
    2.5806161  -3.51874267  3.24561909]'''


    [optimalX, optimalY] = opt.get_maximum()
    print('Optimal Y: ', optimalY)
    print('Optimal X: ', optimalX)
    rbfn = RBFN.RBFN(1, 5, 3, runtime) # indim, numCenters, outdim, time
    rbfn.setHyperParams(optimalX.reshape(5, 3))
    timeInterval = safeopt.linearly_spaced_combinations([(0, runtime)], runtime*10)
    priorities = np.zeros((3, 0))
    for time in timeInterval:
        priorities = np.column_stack((priorities, rbfn.calOutput(time).T))

    # print(priorities)
    plt.figure()
    plt.title('Task Priorities')
    plt.plot(timeInterval,priorities[0], color='r')
    plt.plot(timeInterval,priorities[1], color='g')
    plt.plot(timeInterval,priorities[2], color='b')


    # plot robot states, joint angle, vel, accel
    evaluation = evaluationInter.evaluationInter(runtime=runtime)
    evaluation.updateHyperParams(optimalX)
    evaluation.evaluation()
    JointAngleCurve = evaluation.getJointAngleCurve()
    JointVelocityCurve = evaluation.getJointVelocityCurve()
    JointAccelerationCurve = evaluation.getJointAccelerationCurve()
    plt.figure()
    plt.title('Joint Angle')
    for angle in JointAngleCurve:
        plt.plot(timeInterval, angle)

    plt.figure()
    plt.title('Joint Velocity')
    for vel in JointVelocityCurve:
        plt.plot(timeInterval, vel)

    plt.figure()
    plt.title('Joint Acceleration')
    for accel in JointAccelerationCurve:
        plt.plot(timeInterval, accel)

    plt.ioff()
    print('Optimal Y: ', optimalY)
    print('Optimal X: ', optimalX)
    plt.show()

    '''x_next = np.array([ -6.86710281,   0.32646077,  -5.7733384,   -1.81021915,  -9.66391179,
      -6.14784208,  -0.10733062,  -5.57970494,  -8.68322761, -10.34465487,
       1.70166791,   1.29842112,  -8.83978695,  -2.43083296, -10.04379552])'''



    displayResult = evaluationInter.evaluationInter(runtime=runtime)
    displayResult.updateHyperParams(optimalX)
    print(displayResult.evaluation())
    displayResult.guiStatic()

    '''displayResult = evaluationInter.evaluationInter(runtime=runtime)
    displayResult.updateHyperParams(optimalX)
    displayResult.gui()'''