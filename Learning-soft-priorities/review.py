from singleEpisode import evaluationInter
import learningPriorities
import numpy as np
import matplotlib.pyplot as plt
import safeopt
from RBFN import RBFN

runtime = learningPriorities.runtime
# optimalX = np.array([ -6.86710281,   0.32646077,  -5.7733384,   -1.81021915,  -9.66391179, -6.14784208,  -0.10733062,  -5.57970494,  -8.68322761, -10.34465487, 1.70166791,   1.29842112,  -8.83978695,  -2.43083296, -10.04379552])

'''optimalX = np.array([4.41825932,  0.54252292, -1.37572338, -1.78126762, -2.77853702,  1.85727287,
 -0.90816901, -0.52089679, -1.14786593,  4.64987452, -0.81675974,  7.5770911,
  6.50357604,  2.99004245, -3.51097471])'''

optimalX = np.array([-3.03131596,  0.04519549,  0.05640378,  2.83386375, -2.04397627,  5.82513742,
  2.94663176, -3.23828459,  4.31864194, -3.06726588,  3.35944805,  1.04605141,
  4.87498366, -0.61144085,  0.9063866] )

'''Optimal Y:  [-0.43250899]
Optimal X:  [ 3.47754433  0.06314631  0.41764262 -1.96400342 -1.58139261  4.92449457
 -4.71530316 -2.10776775  1.49432157  3.43835685  2.89568113  0.30602795
  4.18443959  6.11357951 -3.97028203]'''

'''Optimal Y:  [-0.40181033]
Optimal X:  [ 4.41825932  0.54252292 -1.37572338 -1.78126762 -2.77853702  1.85727287
 -0.90816901 -0.52089679 -1.14786593  4.64987452 -0.81675974  7.5770911
  6.50357604  2.99004245 -3.51097471]'''

'''[-3.03131596  0.04519549  0.05640378  2.83386375 -2.04397627  5.82513742
  2.94663176 -3.23828459  4.31864194 -3.06726588  3.35944805  1.04605141
  4.87498366 -0.61144085  0.9063866 ]'''

inputDimension = 15
# optimalX = x0 = np.zeros((1, inputDimension))
rbfn = RBFN.RBFN(1, 5, 3, runtime) # indim, numCenters, outdim, time
rbfn.setHyperParams(optimalX.reshape(5, 3))
timeInterval = safeopt.linearly_spaced_combinations([(0, runtime)], runtime*10)
priorities = np.zeros((3, 0))
for time in timeInterval:
    priorities = np.column_stack((priorities, rbfn.calOutput(time).T))

print(priorities)
plt.figure()
plt.plot(timeInterval,priorities[0], color='r')
plt.plot(timeInterval,priorities[1], color='g')
plt.plot(timeInterval,priorities[2], color='b')
plt.ioff()


displayResult = evaluationInter.evaluationInter(runtime=runtime)
displayResult.updateHyperParams(optimalX)
print(displayResult.evaluation())


JointAngleCurve = displayResult.getJointAngleCurve()
JointTauCurve = displayResult.getJointTauCurve()
JointAccelerationCurve = displayResult.getJointAccelerationCurve()

txt_data = np.column_stack((timeInterval, priorities.T, JointAccelerationCurve.T, JointAngleCurve.T))

np.savetxt("/home/zhou/lee/safeLearningPriorities/mat_plot/robot_state/robotStates.txt", txt_data)
'''np.savetxt("/home/zhou/lee/safeLearningPriorities/mat_plot/robot_state/JointAccelerationCurve.txt", JointAccelerationCurve.T)
np.savetxt("/home/zhou/lee/safeLearningPriorities/mat_plot/robot_state/priorities.txt", priorities.T)
np.savetxt("/home/zhou/lee/safeLearningPriorities/mat_plot/robot_state/timeInterval.txt", timeInterval)'''


plt.figure()
plt.title('Joint Angle')
for angle in JointAngleCurve:
    plt.plot(timeInterval, angle)

plt.figure()
plt.title('Joint Tau')
for tau in JointTauCurve:
    plt.plot(timeInterval, tau)

plt.figure()
plt.title('Joint Acceleration')
for accel in JointAccelerationCurve:
    plt.plot(timeInterval, accel)

plt.show()

print(JointAccelerationCurve)

displayResult.guiStatic()

'''displayResult = evaluationInter.evaluationInter(runtime=runtime)
displayResult.updateHyperParams(optimalX)
displayResult.gui()'''