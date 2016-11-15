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

f, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(x,y0, 'r')
ax1.plot(x,y1, 'g')
ax3 = ax1.twinx()
ax3.plot(x,y2, 'b')
ax3.plot(x,y3, 'y')

for i in range(0,len(y0)):
  y0[i] = float(y0[i]) - float(y1[i])
for i in range(0,len(y2)):
  y2[i] = float(y2[i]) - float(y3[i])

ax2.plot(x,y0, 'r')
ax2.plot(x,y2, 'g')


# fig, ax1 = plt.subplots()

# ax1.set_title("Plot title...")
# ax1.set_xlabel('Time')
# ax1.set_ylabel('Angle')

# ax2 = ax1.twinx()

# ax1.plot(x,y0, 'r')
# ax1.plot(x,y1, 'r')
# ax2.plot(x,y2, 'g')
# ax2.plot(x,y3, 'g')

# leg = ax1.legend()

plt.show()
