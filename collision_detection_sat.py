from ca_utils import *
import matplotlib.pyplot as plt
plt.style.use('default')
from shapely.geometry import Polygon

steps = 100
steps_per_round = 50
radius = 50
m = 1

#obj1_pos_series_x = [radius*math.sin(t*2*math.pi/steps_per_round) for t in range(steps)]
#obj1_pos_series_y = [radius*math.cos(t*2*math.pi/steps_per_round) for t in range(steps)]

obj1_pos_series_x = [50+m*t for t in range(steps)]
obj1_pos_series_y = [150-m*t for t in range(steps)]

obj2_pos_series_x = [50+t for t in range(steps)]
obj2_pos_series_y = [50+m*t for t in range(steps)]

object_1 = moving_object(0,0,0,type="Tractor",rel_x_start=0,rel_y_start=0,external_timestamp=0)
object_2 = moving_object(0,0,1,type="Spraying drone",rel_x_start=45,rel_y_start=0,external_timestamp=0)


figure, ax = plt.subplots(figsize=(10,10))
plt.axis('equal')
plt.axis([-1000, 1000, -1000, 1000])
ax.set_autoscale_on(False)

zero_polygon = ([0,0,0,0],[0,0,0,0])

plt.ion()
p1 = plt.plot(0,0,".",color = "red")[0]
p2 = plt.plot(0,0,".",color = "blue")[0]
r1 = plt.plot(zero_polygon,c = "red")[0]
r2 = plt.plot(zero_polygon,c = "blue")[0]
r1_ss = plt.plot(zero_polygon,"--",c = "red")[0]
r2_ss = plt.plot(zero_polygon,"--", c = "blue")[0]

for i in range(steps):
    object_1.update_position_local(obj1_pos_series_x[i],obj1_pos_series_y[i],external_timestamp=i+1)
    object_2.update_position_local(obj2_pos_series_x[i],obj2_pos_series_y[i],external_timestamp=i+1)
    
    res_str = "Collision time interval: "+str(moving_separating_axis_theorem(object_1, object_2))+"\n"\
        + "Vehicle collision state: "+ str(separating_axis_theorem(object_1,object_2,super_safe=False))+"\n"\
        + "Safe areas collision state: "+str(separating_axis_theorem(object_1, object_2))
    ax.set_title(res_str)
    plot_object_lines(object_1,object_2,r1, r1_ss,r2, r2_ss,p1,p2)
    plt.pause(0.5)
