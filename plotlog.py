import numpy as np
import matplotlib.pyplot as plt

with open("log.txt") as f:
    data = f.read()

data = data.split('\n')
# print data
ll = [row.split('\t') for row in data]
x = ll[:,0]
# print x
y = ll[:,1]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Plot title...")
ax1.set_xlabel('your x label..')
ax1.set_ylabel('your y label...')

ax1.plot(x,y, c='r', label='the data')

leg = ax1.legend()

plt.show()
