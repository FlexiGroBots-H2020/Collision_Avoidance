import numpy as np
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = 4,3
from matplotlib.animation import FuncAnimation

steps = 100
steps_per_round = 50
radius = 30
obj1_pos_series_x = [radius*np.sin(t*2*np.pi/steps_per_round) for t in range(steps)]
obj1_pos_series_y = [radius*np.cos(t*2*np.pi/steps_per_round) for t in range(steps)]


# create a figure with an axes
fig, ax = plt.subplots()
# set the axes limits
ax.axis([-40,40,-40,40])
# set equal aspect such that the circle is not shown as ellipse
ax.set_aspect("equal")
# create a point in the axes
point, = ax.plot(0,1, marker="o")

# Updating function, to be repeatedly called by the animation
def update(i):
    # obtain point coordinates 
    x,y = obj1_pos_series_x[i],obj1_pos_series_y[i]
    # set point's coordinates
    point.set_data([x],[y])
    return point,

# create animation with 10ms interval, which is repeated,
# provide the full circle (0,2pi) as parameters
ani = FuncAnimation(fig, update, interval=100, blit=True, repeat=True,
                    frames=range(steps))

plt.show()