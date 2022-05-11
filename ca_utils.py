# ca_utils

import numpy as np
import math
import time
from geographiclib.geodesic import Geodesic
from haversine import haversine, Unit
from shapely.geometry import Point
from datetimerange import DateTimeRange

##  Defines distances from object, according to heading direction, defining safe rectangle [meters]
safe_dst = {"Tractor":[5,5,5,20], "Cow": [5,5,5,5]}

origin = (65,14)

vertical_line_threshold = 1/1000000

def line_eq_from_points (p1: Point, p2: Point):       ## returns tuple (m,q) for equation y = mx +q, if line is vertical returns (inf, x0) for equation x = x0
    if abs(p1.x-p2.x) > vertical_line_threshold:    ## check if line is not vertical
        m = (p1.y-p2.y)/(p1.x-p2.x)
        q = (p1.x*p2.y-p2.x*p1.y)/(p1.x-p2.x)
    else:
        m = float('inf')
        q = p1.x
    return line(m,q)

class line:     ## line object with equation y = mx + q
    def __init__(self,m,q):
        self.m = m
        self.q = q
    def __str__(self):
        return "Line with equation y = %fx + %f" % (self.m, self.q)

def intersection_between_lines (l1: line, l2: line):
    if l1.m != float('inf') and l2.m != float('inf'):
        a = np.array([[-l1.m,1],[-l2.m,1]])
        b = np.array([l1.q,l2.q])
        x = np.linalg.solve(a,b)
        return Point(x[0],x[1])
    elif l1.m == float('inf') and l2.m != float('inf'):
        return Point(l1.q, (l2.m*l1.q + l2.q))
    elif l2.m == float('inf') and l1.m != float('inf'):
        return Point(l2.q, (l1.m*l2.q + l1.q))
    else:
        return Point(float('inf'),float('inf'))

def cartesian_rotation(p: Point,angle):
    a = math.pi/2 -angle
    x_rotated, y_rotated = (p.x*math.cos(a)-p.y*math.sin(a),p.y*math.cos(a)+p.x*math.sin(a))
    return Point(x_rotated, y_rotated)

def heading_between_two_points(p1: Point, p2: Point):  ## p1 Current Point, p2 Previous Point
    dx = p1.x -p2.x
    dy = p1.y -p2.y
    if (dy) != 0:
        at = math.atan(dx/dy)
        if dy > 0:
            h = at
        elif dx > 0:
            h = at + math.pi
        else:
            h = at - math.pi
    elif dx > 0:
        h = math.pi/2
    else:
        h = -math.pi/2
    return h

def get_relative_coordinates(origin_coordinates, coordinates):
    d = haversine(origin_coordinates,(coordinates), unit=Unit.METERS)
    b = Geodesic.WGS84.Inverse(origin_coordinates[0],origin_coordinates[1],coordinates[0],coordinates[0])['azi1']
    x_rel = d*math.sin(b)
    y_rel = d*math.cos(b)
    return Point(x_rel, y_rel)

class moving_object:
    def __init__(self, lat, lon, id, type, rel_x_start = float("inf"),rel_y_start = float("inf"),external_timestamp = -1):
        self.lat = lat
        self.lon = lon
        if external_timestamp == -1:
            self.timestamp = time.time()
        else:
            self.timestamp = external_timestamp
        self.type = type
        self.id = id
        if rel_x_start == float("inf") or rel_y_start == float("inf"):
            self.xy = get_relative_coordinates(origin, (self.lat, self.lon))
        else:
            self.xy = Point(rel_x_start,rel_y_start)
        self.heading = 0
        self.vertices = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        self.counter = 0
        pass

    def update_direction_lines(self):
        self.r_l = line_eq_from_points(self.vertices[0], self.vertices[3])
        self.r_r = line_eq_from_points(self.vertices[1], self.vertices[2])
        pass

    def update_safety_rectangle(self):
        vert = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        vert[0] = cartesian_rotation(Point(-safe_dst[self.type][1],safe_dst[self.type][0]),self.heading)
        vert[1] = cartesian_rotation(Point(-safe_dst[self.type][1],-safe_dst[self.type][2]),self.heading)
        vert[2] = cartesian_rotation(Point(safe_dst[self.type][3],-safe_dst[self.type][2]),self.heading)
        vert[3] = cartesian_rotation(Point(safe_dst[self.type][3],safe_dst[self.type][0]),self.heading)
        self.vertices = [Point(vert[i].x+self.xy.x,vert[i].y+self.xy.y) for i in range(4)]
        pass


    def update_position(self,n_lat, n_lon,external_timestamp = -1):
        self.prev_lat = self.lat
        self.lat = n_lat
        self.prev_lon = self.lon
        self.lon = n_lon
        self.prev_timestamp = self.timestamp

        if external_timestamp == -1:
            self.timestamp = time.time()
        else:
            self.timestamp = external_timestamp

        distance = haversine((self.lat, self.lon),(self.prev_lat,self.prev_lon), unit=Unit.METERS)
        self.speed = distance / ((self.timestamp - self.prev_timestamp))        ##speed in m/s
        self.xy_prev = self.xy
        self.xy = get_relative_coordinates(origin, (self.lat, self.lon))
        self.heading = heading_between_two_points(self.xy, self.xy_prev)
        self.heading_deg = self.heading*180/math.pi
        self.counter +=1
        pass

    def update_position_local(self, t_x, t_y, external_timestamp = -1):
        self.xy_prev = self.xy
        self.xy = Point(t_x, t_y)
        self.prev_timestamp = self.timestamp

        if external_timestamp == -1:
            self.timestamp = time.time()
        else:
            self.timestamp = external_timestamp

        distance = self.xy.distance(self.xy_prev)
        self.speed = distance / ((self.timestamp - self.prev_timestamp))        ##speed in m/s
        self.heading = heading_between_two_points(self.xy, self.xy_prev)
        self.heading_deg = self.heading*180/math.pi
        self.counter +=1
        pass

##  Collision Avoidance function

def collision_state(obj_1: moving_object, obj_2: moving_object):
    collision = False
    if obj_1.counter == 0 or obj_2.counter == 0:
        return "Impossible to calculate collision time without previous positions"

    ## compute directional lines
    r1_l = obj_1.r_l
    r1_r = obj_1.r_r
    r2_l = obj_2.r_l
    r2_r = obj_2.r_r
    

    ## compute intersection points (c_xy with x side from obj1 and y side form obj2)
    c_ll = intersection_between_lines(r1_l, r2_l)
    c_lr = intersection_between_lines(r1_l, r2_r)
    c_rl = intersection_between_lines(r1_r, r2_l)
    c_rr = intersection_between_lines(r1_r, r2_r)
    c_list = [c_ll,c_lr,c_rr,c_rl]
    
    collision_heading_1 = heading_between_two_points(c_ll, obj_1.vertices[0])
    print(round(collision_heading_1*(180/math.pi),5),round(obj_1.heading_deg,5))

    collision_heading_2 = heading_between_two_points(c_ll, obj_2.vertices[0])
    print(round(collision_heading_2*(180/math.pi),5),round(obj_2.heading_deg,5))

    if round(collision_heading_1,5) != round(obj_1.heading,5) or round(collision_heading_2,5) != round(obj_2.heading,5):
        print("No collision")
    else:
        collision = True

    collision_time_table_1 = np.zeros((4,4))
    collision_time_table_2 = np.zeros((4,4))

    for i in range(4):
        for j in range(4):
            collision_time_table_1[i,j] = obj_1.vertices[i].distance(c_list[j])/obj_1.speed
            collision_time_table_2[i,j] = obj_2.vertices[i].distance(c_list[j])/obj_2.speed

    occupation_timedelta_1 = DateTimeRange(collision_time_table_1.min(),collision_time_table_1.max())
    occupation_timedelta_2 = DateTimeRange(collision_time_table_2.min(),collision_time_table_2.max())
    collision_time = max(collision_time_table_1.min(),collision_time_table_2.min())
    if occupation_timedelta_1.is_intersection(occupation_timedelta_2) and collision == True:
        print("collision in %f seconds" % collision_time)
        return [c_list, collision_time_table_1, collision_time_table_2,occupation_timedelta_1,occupation_timedelta_2]
    else:
        print("No collision")
        return [c_list]
    

def collision_sat(obj_1: moving_object, obj_2: moving_object):
    x1_0 = min([vertices.x for vertices in obj_1.vertices])
    x1_1 = max([vertices.x for vertices in obj_1.vertices])
    x2_0 = min([vertices.x for vertices in obj_2.vertices])
    x2_1 = max([vertices.x for vertices in obj_2.vertices])

    y1_0 = min([vertices.y for vertices in obj_1.vertices])
    y1_1 = max([vertices.y for vertices in obj_1.vertices])
    y2_0 = min([vertices.y for vertices in obj_2.vertices])
    y2_1 = max([vertices.y for vertices in obj_2.vertices])

    v1_x = round(obj_1.speed*math.sin(obj_1.heading),5)
    v1_y = round(obj_1.speed*math.cos(obj_1.heading),5)

    v2_x = round(obj_2.speed*math.sin(obj_2.heading),5)
    v2_y = round(obj_2.speed*math.cos(obj_2.heading),5)
    
    print("speed components")
    print(v1_x, v1_y)
    print(v2_x, v2_y)

    
    if (v1_x - v2_x) != 0:
        s0x = [((x2_0-x1_0)/(v1_x-v2_x)),((x2_1-x1_0)/(v1_x-v2_x))]
        s1x = [((x1_0-x2_0)/(v2_x-v1_x)),((x1_1-x2_0)/(v2_x-v1_x))]
        ci_x = [min(s0x+s1x),max(s0x+s1x)]
    else:
        ci_x = [-float("inf"),float("inf")]

    if (v1_y - v2_y) != 0:
        s0y = [((y2_0-y1_0)/(v1_y-v2_y)),((y2_1-y1_0)/(v1_y-v2_y))]
        s1y = [((y1_0-y2_0)/(v2_y-v1_y)),((y1_1-y2_0)/(v2_y-v1_y))]
        ci_y = [min(s0y+s1y),max(s0y+s1y)]
    else:
        ci_y = [-float("inf"),float("inf")]

    print("crossing intervals")
    print(ci_x)
    print(ci_y)

    if (min(ci_x[1], ci_y[1]) - max(ci_x[0], ci_y[0])) > 0:
        time_to_collision = max(ci_x[0], ci_y[0])
        time_end_collision = min(ci_x[1], ci_y[1])
        
        return [time_to_collision, time_end_collision]



    return "Ao"