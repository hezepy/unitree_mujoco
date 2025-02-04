import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('log.txt')
t = data[:, 0]
x = data[:, 16]
y = data[:, 36]

# data = data.split('\n')

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Joint torque")    
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Joint torque [Nm]')

ax1.plot(t,x, c='r', label='ref. torq.')
ax1.plot(t,y, c='b', label='act. torq.')

leg = ax1.legend()

plt.show()

