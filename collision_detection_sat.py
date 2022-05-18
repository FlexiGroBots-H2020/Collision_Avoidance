from ca_utils import *
from shapely.geometry import Point, Polygon, MultiPoint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
plt.style.use('default')
import time

steps = 100
steps_per_round = 50
radius = 50
m = 1

obj1_pos_series_x = [radius*math.sin(t*2*math.pi/steps_per_round) for t in range(steps)]
obj1_pos_series_y = [radius*math.cos(t*2*math.pi/steps_per_round) for t in range(steps)]

#obj1_pos_series_x = [m*t for t in range(steps)]
#obj1_pos_series_y = [50-m*t for t in range(steps)]

obj2_pos_series_x = [-50+t for t in range(steps)]
obj2_pos_series_y = [-50+m*t for t in range(steps)]

object_1 = moving_object(0,0,0,type="Tractor",rel_x_start=0,rel_y_start=0,external_timestamp=0)
object_2 = moving_object(0,0,1,type="Spraying drone",rel_x_start=45,rel_y_start=0,external_timestamp=0)

figure, ax = plt.subplots(figsize=(5,5))
plt.axis('equal')
plt.ion()
plt.xlim([-100, 100])
plt.ylim([-100, 100])
ax.set_autoscale_on(False)

for i in range(steps):
    object_1.update_position_local(obj1_pos_series_x[i],obj1_pos_series_y[i],external_timestamp=i+1)
    object_2.update_position_local(obj2_pos_series_x[i],obj2_pos_series_y[i],external_timestamp=i+1)
    plt.text(-50,-50,str(moving_separating_axis_theorem(object_1, object_2)))
    plt.text(50,50,str(separating_axis_theorem(object_1, object_2)))
    plot_object_lines(object_1,object_2,ax,figure)
    time.sleep(1/25)