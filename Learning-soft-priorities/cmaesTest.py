from pycma import cma
import learningPriorities
import matplotlib.pyplot as plt
import numpy as np


'''a=np.array([i for i in range(1000)]).reshape((1000,1))
print(a.shape)
b=np.column_stack((a,a+1))
print(b)
np.savetxt("haha.txt", b)'''


plt.close()
plt.ion()
x0 = learningPriorities.cmaes_initiateSafeSet(learningPriorities.inputDimension)

print(x0)

es = cma.CMAEvolutionStrategy(x0[0], 0.01)
i=0
while not es.stop():
    solutions = es.ask()
    es.tell(solutions, [learningPriorities.cmaesf(x) for x in solutions])
    es.logger.add()  # write data to disc to be plotted
    es.disp()
    es.plot()
    plt.pause(0.01)
    i=i+1
    print('order')
    print(i)
    if i > 66:
        exit()

print('ending')