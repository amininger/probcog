#!/usr/bin/env python3

# Info File Format Specification:

# robot <x> <y>
#   The start location of the robot

# region <id> <x> <y> <rot> <width> <height> <category>
#   Defines a rectangular region centered at the given coordinates

# edge <start_region_id> <end_region_id> open
#   Defines a connection between the two regions (can drive between them)
#   This assumes you can get from any point in region 1 to the center of region 2 with a straight line (no obstacles)
# edge <start_region_id> <end_region_id> door <doorx> <doory> <doortheta>
#   Defines a connection between the two regions, but with a given intermediate waypoint (door)
#   The doortheta is the angle to drive along when going from start_region to end_region

# wall <sx> <sy> <ex> <ey>
#   Defines a wall segment from the start coordinate to the end coordinate

# wallchain <num_segs>
#   <x1> <y1>
#   <x2> <y2> 
#   ...
#   <xn> <yn>
#   <xn+1> <yn+1>

#   Defines a chain of N connected wall segments, 
#     where the i'th segment goes from coordinate i to i+1 

# object <x> <y> <z> <r> <p> <y> <sx> <sy> <sz> <r> <g> <b> <num-preds> <pred1> <prop1> <pred2> <prop2> ... <label>
#   Defines an object in the environment (represented as a bounding box)
#   xyz - position, rpy - rotation, sx/sy/sz - scale of bounding box (1.00 = side length of 1m), rgb - color
#   predicates are any number of key-value pairs describing the object
# Examples:
#   object 8.00 14.0 1.00 0 0 0 0.50 0.50 2.00 255 200 100 2 name alice1 category person Alice90 
#   object 4.70 0.20 1.20 0 0 0 0.60 0.40 0.40 40 40 40 4 category microwave1 activation1 off2 door2 closed2 receptacle receptacle Microwave9 
#   object 1.50 13.5 1.05 0 0 0 0.40 0.30 0.10 0 0 0 2 category book1 color black1 Book42 



import sys
from file_reader import FileReader

from math import *

def parse_info_file(name, info_filename):
    fin = open(info_filename, 'r')
    reader = FileReader(fin)
    world_info = WorldInfo(reader, name)
    fin.close()
    return world_info

#SCALE_FACTOR = 0.254 # in to decimeter
SCALE_FACTOR = 1.0 # Magicbot - input is in meters

####### ALL INFO ######

class WorldInfo:
    def __init__(self, reader, name):
        self.name = name
        self.robot = None
        self.walls = []
        self.regions = []
        self.edges = []
        self.doors = []
        self.objects = []

        # Parse each item
        word = reader.nextWord()
        while word != None:
            try:
                itemtype = word
                if itemtype == "robot":
                    self.robot = RobotInfo().read_info(reader)
                elif itemtype == "region":
                    self.regions.append(RegionInfo().read_info(reader))
                elif itemtype == "edge":
                    self.edges.append(EdgeInfo().read_info(reader))
                elif itemtype == "wall":
                    self.walls.append(WallInfo().read_info(reader))
                elif itemtype == "wallchain":
                    self.walls.extend(parseWallChain(reader))
                elif itemtype == "object":
                    self.objects.append(ObjectInfo().read_info(reader))
                word = reader.nextWord()
            except:
                raise Exception("Parsing Error in line " + str(reader.lineNum))

###### ROBOT ######

class RobotInfo:
    def __init__(self):
        self.x = 0
        self.y = 0

    def read_info(self, reader):
        self.x = float(reader.nextWord()) * SCALE_FACTOR
        self.y = float(reader.nextWord()) * SCALE_FACTOR
        return self

###### REGION ######

class RegionInfo:
    def __init__(self):
        self.tag_id = 0
        self.x = 0
        self.y = 0
        self.rot = 0
        self.width = 0
        self.length = 0
        self.label = None

    def read_info(self, reader):
        self.tag_id = int(reader.nextWord())

        # Note: this assumes all ids are less than 100 (and thus 2 characters)
        #   Increase this to accomodate more node ids
        self.soar_id = str(self.tag_id)
        while len(self.soar_id) < 2:
            self.soar_id = "0" + self.soar_id

        self.x = float(reader.nextWord()) * SCALE_FACTOR
        self.y = float(reader.nextWord()) * SCALE_FACTOR
        self.rot = float(reader.nextWord())
        self.width = float(reader.nextWord()) * SCALE_FACTOR
        self.length = float(reader.nextWord()) * SCALE_FACTOR
        self.label = reader.nextWord()
        return self

    def contains_point(self, x, y):
        dx = self.x - x
        dy = self.y - y
        dist = sqrt(dx*dx + dy*dy)
        theta = atan2(dy, dx)
        local_theta = theta - self.rot
        xproj = dist * cos(local_theta)
        yproj = dist * sin(local_theta)
        return abs(xproj) < self.width/2 and abs(yproj) < self.length/2

###### EDGES ######

class EdgeInfo:
    def __init__(self):
        self.start_wp = 0
        self.end_wp = 0
        self.has_door = False
    
    def read_info(self, reader):
        self.start_wp = int(reader.nextWord())
        self.end_wp = int(reader.nextWord())
        label = reader.nextWord()
        if label == "open":
            self.has_door = False
        elif label == "door":
            self.has_door = True
            self.door_x = float(reader.nextWord()) * SCALE_FACTOR
            self.door_y = float(reader.nextWord()) * SCALE_FACTOR
            self.door_rot = float(reader.nextWord())
        return self

###### WALLS ######

class WallInfo:
    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0

    def read_info(self, reader):
        self.x1 = float(reader.nextWord()) * SCALE_FACTOR
        self.y1 = float(reader.nextWord()) * SCALE_FACTOR
        self.x2 = float(reader.nextWord()) * SCALE_FACTOR
        self.y2 = float(reader.nextWord()) * SCALE_FACTOR
        return self

def parseWallChain(reader):
    walls = []
    n = int(reader.nextWord())
    x1 = float(reader.nextWord()) * SCALE_FACTOR
    y1 = float(reader.nextWord()) * SCALE_FACTOR
    for i in range(n):
        x2 = float(reader.nextWord()) * SCALE_FACTOR
        y2 = float(reader.nextWord()) * SCALE_FACTOR
        wall = WallInfo()
        wall.x1 = x1
        wall.y1 = y1
        wall.x2 = x2
        wall.y2 = y2
        walls.append(wall)
        x1 = x2
        y1 = y2
    return walls

###### DOORS #####

class DoorInfo:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.wp1 = 0
        self.wp2 = 0
        self.open = True

    def read_info(self, reader):
        self.x = float(reader.nextWord()) * SCALE_FACTOR
        self.y = float(reader.nextWord()) * SCALE_FACTOR
        self.yaw = float(reader.nextWord())
        self.wp1 = int(reader.nextWord())
        self.wp2 = int(reader.nextWord())
        self.open = (True if reader.nextWord() == "open" else False)
        return self

###### OBJECTS ######

class ObjectInfo:
    next_id = 1
    def __init__(self):
        self.vals = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.rgb = [ 0, 0, 0 ]
        self.cats = []
        self.labels = []
        self.desc = ""

    def read_info(self, reader):
        self.obj_id = ObjectInfo.next_id
        ObjectInfo.next_id += 1
        for i in range(0, 3):
            self.vals[i] = float(reader.nextWord()) * SCALE_FACTOR
        for i in range(3, 6):
            self.vals[i] = float(reader.nextWord()) 
        for i in range(6, 9):
            self.vals[i] = float(reader.nextWord()) * SCALE_FACTOR
        self.rgb = []
        self.rgb.append(int(reader.nextWord()))
        self.rgb.append(int(reader.nextWord()))
        self.rgb.append(int(reader.nextWord()))

        num_labels = int(reader.nextWord())
        self.cats = []
        self.labels = []
        for i in range(num_labels):
            self.cats.append(reader.nextWord())
            self.labels.append(reader.nextWord())
        self.desc = reader.nextWord()
        return self

