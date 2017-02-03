import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from collections import deque
import serial

style.use('fivethirtyeight')

fig = plt.figure()

ax1 = fig.add_subplot(1,1,1)

maxLen = 50
ys = deque([0.0]*maxLen)
xs = range(maxLen)

ard = serial.Serial('/dev/ttyACM1', 115200)

def animate(i):	
	ardString = ard.readline()
	try:
		float(ardString.split(' ')[0])
	except:
		ardString = ard.readline()

	ard.flush()
	ard.flushInput()
	ard.flushOutput()
	val = float(ardString.split(' ')[0])
	
	if len(xs) < maxLen:
		ys.append(val)
	else:
		ys.pop()
        ys.appendleft(val)

	ax1.clear()
	plt.ylim((-0.5,0.5))
	ax1.plot(xs,ys);

ani = animation.FuncAnimation(fig, animate, interval=50)

plt.show()