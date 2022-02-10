#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

time_array = np.linspace(0, 2*np.pi, 100)
print(time_array)

r = 0.1
x = r*np.cos(time_array)
y = np.sin(time_array)

fig, ax = plt.subplots(1)

plt.xlim(-2.5, 2.5)
plt.ylim(-2.5, 2.5)

ax.plot(x, y)
plt.grid(linestyle='--')
plt.title('Drone Path', fontsize=8)
plt.show()