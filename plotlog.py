import numpy as np
import matplotlib.pyplot as plt

with open("log.txt") as f:
    data = f.read()

data = data.split('\n')
del data[-1]
x = [row.split('\t')[0] for row in data]
del x[-1]
y0 = [row.split('\t')[1] for row in data]
del y0[-1]
y1 = [row.split('\t')[2] for row in data]
del y1[-1]
y2 = [row.split('\t')[3] for row in data]
del y2[-1]
y3 = [row.split('\t')[4] for row in data]
del y3[-1]

fig, ax1 = plt.subplots()

ax1.set_title("Plot title...")
ax1.set_xlabel('Time')
ax1.set_ylabel('Angle')

ax2 = ax1.twinx()

ax1.plot(x,y0, 'r')
ax1.plot(x,y1, 'r')
ax2.plot(x,y2, 'g')
ax2.plot(x,y3, 'g')

leg = ax1.legend()

plt.show()
