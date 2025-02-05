import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import argparse
from pathlib import Path

# parser = argparse.ArgumentParser()
# parser.add_argument("file_path", type=Path)

# p = parser.parse_args()
# print(p.file_path, type(p.file_path), p.file_path.exists())

data = np.loadtxt('build/logs/2025_02_05_14_57_50/log.txt')
t = data[:, 0]
x = data[:, 3]
y = data[:, 23]

# data = data.split('\n')

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Joint torque")    
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Joint torque [Nm]')

ax1.plot(t,y, c='b', label='act. torq.')
ax1.plot(t,x, c='r', label='ref. torq.')

leg = ax1.legend()

plt.show()

