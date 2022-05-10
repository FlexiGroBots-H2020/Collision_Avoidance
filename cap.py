from ca_utils import *
import shapely
import matplotlib.pyplot as plt
object_1 = moving_object(0,0,0,type="Tractor")
object_2 = moving_object(0,0,1,type="Cow")
time.sleep(0.001)
object_2.update_position_local(100,0)
time.sleep(1)
object_1.update_position_local(5,5)
object_2.update_position_local(95,5)
print(object_1.speed)
print(object_1.timestamp - object_1.prev_timestamp)
print((object_1.heading*(180/math.pi)),(object_2.heading*(180/math.pi)))
object_1.update_safety_rectangle()
object_2.update_safety_rectangle()
res = collision_state(object_1,object_2)
cl = res[0]
ct1 = res[1]
ct2 = res[2]
otd1 = res[3]
otd2 = res[4]
g = shapely.geometry.MultiPoint(object_1.vertices+[object_1.xy]+object_2.vertices+[object_2.xy]+cl)
xs = [point.x for point in g]
ys = [point.y for point in g]
plt.scatter(xs, ys)
plt.show()