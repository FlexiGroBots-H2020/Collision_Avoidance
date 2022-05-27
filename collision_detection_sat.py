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

obj1_pos_series_x = [m*t for t in range(steps)]
obj1_pos_series_y = [50-m*t for t in range(steps)]

obj2_pos_series_x = [-50+t for t in range(steps)]
obj2_pos_series_y = [-50+m*t for t in range(steps)]

object_1 = moving_object(0,0,0,type="Tractor",rel_x_start=0,rel_y_start=0,external_timestamp=0)
object_2 = moving_object(0,0,1,type="Spraying drone",rel_x_start=45,rel_y_start=0,external_timestamp=0)


figure, ax = plt.subplots(figsize=(10,10))
plt.axis('equal')
plt.axis([-1000, 1000, -1000, 1000])
ax.set_autoscale_on(False)

plt.ion()
p1 = plt.plot(0,0,".",color = "red")[0]
p2 = plt.plot(0,0,".",color = "blue")[0]
r1 = plt.plot(Polygon(object_1.vertices).exterior.xy,c = "red")[0]
r2 = plt.plot(Polygon(object_2.vertices).exterior.xy,c = "blue")[0]
r1_ss = plt.plot(Polygon(object_1.super_safe_vertices).exterior.xy,c = "red")[0]
r2_ss = plt.plot(Polygon(object_2.super_safe_vertices).exterior.xy, c = "blue")[0]

for i in range(steps):
    object_1.update_position_local(obj1_pos_series_x[i],obj1_pos_series_y[i],external_timestamp=i+1)
    object_2.update_position_local(obj2_pos_series_x[i],obj2_pos_series_y[i],external_timestamp=i+1)
    
    ax.set_title(""+str(moving_separating_axis_theorem(object_1, object_2))+"\n"+ str(separating_axis_theorem(object_1, object_2)))
    # plt.text(-50,-50,str(moving_separating_axis_theorem(object_1, object_2)))
    # plt.text(50,50,str(separating_axis_theorem(object_1, object_2)))
    plot_object_lines(object_1,object_2,r1, r1_ss,r2, r2_ss,p1,p2)
    plt.pause(0.5)
