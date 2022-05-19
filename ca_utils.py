# ca_utils

import numpy as np
import math
import time
from shapely.geometry import Point,Polygon
import matplotlib.pyplot as plt
import pymap3d as pm


##  Defines distances from object, according to heading direction, defining safe rectangle [meters]
vehicle_priorities = {"Tractor": 10, "Spraying drone":8}
safe_dst = {"Tractor":[2.5,2.5,2.5,4],"Spraying drone": [1,1,1,1] , "Cow": [2,2,2,2]}
super_safe_distances = {"Tractor":[5,5,5,10],"Cow": [5,5,5,5],"Spraying drone": [3,3,3,3]}
origin = (65.0566799,25.4587279)

vertical_line_threshold = 1/1000000


def line_eq_from_points (p1: Point, p2: Point):       # return: line object line(m,q) for equation y = mx +q, if line is vertical returns (inf, x0) for equation x = x0
    if abs(p1.x-p2.x) > vertical_line_threshold:    ## check if line is not vertical
        m = (p1.y-p2.y)/(p1.x-p2.x)
        q = (p1.x*p2.y-p2.x*p1.y)/(p1.x-p2.x)
    else:
        m = float('inf')
        q = p1.x
    return line(m,q)

class line:     ## line(m,q) object with equation y = mx + q, if line is vertical: line(inf, x0) for equation x = x0
    def __init__(self,m,q):
        self.m = m
        self.q = q
    def __str__(self):
        return "Line with equation y = %fx + %f" % (self.m, self.q)

class vector:       # vector from (x0, y0) to (x,y), default (x0, y0) = (0, 0)
    def __init__(self, x, y, x0 = 0, y0 = 0):
        self.x = x
        self.y = y
        self.x0 = x0
        self.y0 = y0

def intersection_between_lines (l1: line, l2: line):    # return: intersection point between two lines
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

def cartesian_rotation(p: Point,angle):     # return: input point rotated around (0,0), angle positive CW from vertical
    a = math.pi/2 -angle
    x_rotated, y_rotated = (p.x*math.cos(a)-p.y*math.sin(a),p.y*math.cos(a)+p.x*math.sin(a))
    return Point(x_rotated, y_rotated)

def heading_between_two_points(p1: Point, p2: Point):   # return: heading of vector from p1 to p2, angle positive CW from vertical
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

def get_relative_coordinates(origin_coordinates, coordinates):      # return: local coordinates Point(x,y) from GPS coordinates and local origin reference point
    ned = pm.geodetic2ned(coordinates[0],coordinates[1],0,origin_coordinates[0],origin_coordinates[1],0)
    y_rel = ned[0]
    x_rel = ned[1]
    return Point(x_rel, y_rel)

class moving_object:    # come se si potesse commentare in una riga
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
        self.xy_prev = self.xy
        self.heading = 0
        self.vertices = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        self.super_safe_vertices = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        self.counter = 0
        self.speed = 0
        pass

    def update_direction_lines(self):   # update direction lines equation (only for rectangles (for now))
        self.r_l = line_eq_from_points(self.vertices[0], self.vertices[3])
        self.r_r = line_eq_from_points(self.vertices[1], self.vertices[2])
        pass

    def update_safety_rectangle(self):  # update coordinates of safety rectangle according to position and heading
        vert = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        vert[0] = cartesian_rotation(Point(-safe_dst[self.type][1],safe_dst[self.type][0]),self.heading)
        vert[1] = cartesian_rotation(Point(-safe_dst[self.type][1],-safe_dst[self.type][2]),self.heading)
        vert[2] = cartesian_rotation(Point(safe_dst[self.type][3],-safe_dst[self.type][2]),self.heading)
        vert[3] = cartesian_rotation(Point(safe_dst[self.type][3],safe_dst[self.type][0]),self.heading)
        self.vertices = [Point(vert[i].x+self.xy.x,vert[i].y+self.xy.y) for i in range(4)]
        pass

    def update_super_safety_rectangle(self):  # update coordinates of safety rectangle according to position and heading
        vert = [Point(0,0),Point(0,0),Point(0,0),Point(0,0)]
        vert[0] = cartesian_rotation(Point(-super_safe_distances[self.type][1],super_safe_distances[self.type][0]),self.heading)
        vert[1] = cartesian_rotation(Point(-super_safe_distances[self.type][1],-super_safe_distances[self.type][2]),self.heading)
        vert[2] = cartesian_rotation(Point(super_safe_distances[self.type][3],-super_safe_distances[self.type][2]),self.heading)
        vert[3] = cartesian_rotation(Point(super_safe_distances[self.type][3],super_safe_distances[self.type][0]),self.heading)
        self.super_safe_vertices = [Point(vert[i].x+self.xy.x,vert[i].y+self.xy.y) for i in range(4)]
        pass

    def update_position(self,n_lat, n_lon,external_heading,external_timestamp = -1):     # update positional parameters from GPS coordinates
        self.prev_lat = self.lat
        self.lat = n_lat
        self.prev_lon = self.lon
        self.lon = n_lon
        self.prev_timestamp = self.timestamp

        if external_timestamp == -1:
            self.timestamp = time.time()
        else:
            self.timestamp = external_timestamp

        self.xy_prev = self.xy
        self.xy = get_relative_coordinates(origin, (self.lat, self.lon))
        distance = self.xy.distance(self.xy_prev)
        self.speed = distance / ((self.timestamp - self.prev_timestamp))
        if self.speed < 0.1:
            self.heading = external_heading*math.pi/180 if external_heading < 180 else (external_heading-360)*math.pi/180
        else:
            self.heading = heading_between_two_points(self.xy, self.xy_prev)
        self.heading_deg = self.heading*180/math.pi
        self.counter +=1
        self.update_super_safety_rectangle()
        self.update_safety_rectangle()
        self.update_direction_lines()
        pass

    def update_position_local(self, t_x, t_y, external_timestamp = -1):     # update positional parameters from local coordinates (for test & debug)
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
        self.update_super_safety_rectangle()
        self.update_safety_rectangle()
        self.update_direction_lines()
        pass

def collision_sat_old(obj_1: moving_object, obj_2: moving_object, verbose = False):     # return: collision time interval [start of collision, end of collision] (using minimum circumscribed rectangle) 
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
    
    if verbose:
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
    if verbose:
        print("crossing intervals")
        print(ci_x)
        print(ci_y)

    if (min(ci_x[1], ci_y[1]) - max(ci_x[0], ci_y[0])) > 0:
        time_to_collision = max(ci_x[0], ci_y[0])
        time_end_collision = min(ci_x[1], ci_y[1])
        
        return [time_to_collision, time_end_collision]

    return [float("inf"),float("inf")]

def plot_object_lines(object_1: moving_object, object_2: moving_object,r1,r1_ss, r2, r2_ss,p1,p2):      # garbage just for demonstation
     # if object_2.r_l.m == float('inf'):
    #     ax.axvline(x = object_2.r_l.q, color="blue", linestyle="--")
    # else:
    #     ax.axline((0,object_2.r_l.q),slope=object_2.r_l.m,color="blue", linestyle="--")

    # if object_2.r_r.m == float('inf'):
    #     ax.axvline(x = object_2.r_r.q, color="blue", linestyle="--")
    # else:
    #     ax.axline((0,object_2.r_r.q),slope=object_2.r_r.m,color="blue", linestyle="--")

    # if object_1.r_l.m == float('inf'):
    #     ax.axvline(x = object_1.r_l.q, color="red", linestyle="--")
    # else:
    #     ax.axline((0,object_1.r_l.q),slope=object_1.r_l.m,color="red", linestyle="--")

    # if object_1.r_r.m == float('inf'):
    #     ax.axvline(x = object_1.r_r.q, color="red", linestyle="--")
    # else:
    #     ax.axline((0,object_1.r_r.q),slope=object_1.r_r.m,color="red", linestyle="--")

    r1.set_data(Polygon(object_1.vertices).exterior.xy)
    r1_ss.set_data(Polygon(object_1.super_safe_vertices).exterior.xy)
    r2.set_data(Polygon(object_2.vertices).exterior.xy)
    r2_ss.set_data(Polygon(object_2.super_safe_vertices).exterior.xy)

    p1.set_data(object_1.xy.x,object_1.xy.y)
    p2.set_data(object_2.xy.x,object_2.xy.y)
   

def normalize(v: vector):      # return: The vector scaled to a length of 1
    norm = math.sqrt(v.x ** 2 + v.y ** 2)
    return vector(v.x / norm, v.y / norm) if norm != 0 else vector(1,0)

def dot(v1: vector, v2: vector):  # return: The dot (or scalar) product of the two vectors (vectors represended as (0,0)-> Point(x,y))
    d = v1.x * v2.x + v1.y * v2.y
    return d

def edge_direction(point0: Point, point1: Point):   # return: A vector going from point0 to point1
    return vector(point1.x - point0.x, point1.y - point0.y)

def orthogonal(vo: vector):     # return: A new vector which is orthogonal to the given vector
    return vector(vo.y, -vo.x)

def vertices_to_edges(vertices):    # return: A list of the edges of the vertices as vectors
    return [edge_direction(vertices[i], vertices[(i + 1) % len(vertices)])for i in range(len(vertices))]

def project(vertices, axis):    # return: A vector showing how much of the vertices lies along the axis
    dots = [dot(vertex, axis) for vertex in vertices]
    return [min(dots), max(dots)]

def interval_intersection(intervals: list):     # return: Intersection between closed & bounded intervals
    p = [-float("inf"),float("inf")]
    for i in range(len(intervals)):
        if (min(intervals[i][1],p[1]) - max(intervals[i][0],p[0])) > 0:
            p = [max(intervals[i][0],p[0]),min(intervals[i][1],p[1])]
        else:
            return [float("nan"),float("nan")]
    return p
    
def local_overlapping_interval(p1:list, p2: list, pv1: float, pv2:float):       # return: Intersection time interval for two moving intervals p1 = [p1_0 + t*v1, p1_1 + t*v1] and [p2_0 + t*v2, p2_1 + t*v2]
    if (pv1 - pv2) != 0:
        s0x = [((p2[0]-p1[0])/(pv1-pv2)),((p2[1]-p1[0])/(pv1-pv2))]
        s1x = [((p1[0]-p2[0])/(pv2-pv1)),((p1[1]-p2[0])/(pv2-pv1))]
        return [min(s0x+s1x),max(s0x+s1x)]
    else:
        return [-float("inf"),float("inf")]

def moving_separating_axis_theorem(obj_a: moving_object, obj_b: moving_object):        # return: Collision time interval [start of collision, end of collision]
    vertices_a = obj_a.vertices
    vertices_b = obj_b.vertices

    va_x = round(obj_a.speed*math.sin(obj_a.heading),5)
    va_y = round(obj_a.speed*math.cos(obj_a.heading),5)
    va = vector(va_x,va_y)
    vb_x = round(obj_b.speed*math.sin(obj_b.heading),5)
    vb_y = round(obj_b.speed*math.cos(obj_b.heading),5)
    vb = vector(vb_x,vb_y)

    edges = vertices_to_edges(vertices_a) + vertices_to_edges(vertices_b)
    axes = [normalize(orthogonal(edge)) for edge in edges]

    overlapping_intervals = []

    for axis in axes:
        projection_a = project(vertices_a, axis)
        projection_b = project(vertices_b, axis)
        speed_projection_a = dot(va,axis)
        speed_projection_b = dot(vb,axis)
        loi = local_overlapping_interval(projection_a,projection_b,speed_projection_a,speed_projection_b)
        overlapping_intervals.append(loi)

    collision_interval = interval_intersection(overlapping_intervals)

    return collision_interval

def separating_axis_theorem(obj_a: moving_object, obj_b: moving_object,super_safe = True):        # return: True if objects are colliding
    if super_safe:
        vertices_a = obj_a.super_safe_vertices
        vertices_b = obj_b.super_safe_vertices
    else:
        vertices_a = obj_a.vertices
        vertices_b = obj_b.vertices
        
    
    edges = vertices_to_edges(vertices_a) + vertices_to_edges(vertices_b)
    axes = [normalize(orthogonal(edge)) for edge in edges]

    for axis in axes:
        projection_a = project(vertices_a, axis)
        projection_b = project(vertices_b, axis)

        oi = interval_intersection([projection_a, projection_b])

        if math.isnan(oi[0]) and math.isnan(oi[1]):
            return False

    return True