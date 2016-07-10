#!/usr/bin/env python3

import sys
from file_reader import FileReader

def parse_info_file(info_filename):
	fin = open(info_filename, 'r')
	reader = FileReader(fin)
	world_info = WorldInfo(reader)
	fin.close()
	return world_info

####### ALL INFO ######

class WorldInfo:
	def __init__(self, reader):
		self.robot = None
		self.walls = []
		self.regions = []
		self.edges = []
		self.doors = []
		self.objects = []

		# Parse each item
		word = reader.nextWord()
		while word != None:
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

###### ROBOT ######

class RobotInfo:
	def __init__(self):
		self.x = 0
		self.y = 0

	def read_info(self, reader):
		self.x = float(reader.nextWord())
		self.y = float(reader.nextWord())
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

		self.x = float(reader.nextWord())
		self.y = float(reader.nextWord())
		self.rot = float(reader.nextWord())
		self.width = float(reader.nextWord())
		self.length = float(reader.nextWord())
		label = reader.nextWord()
		if label == "none":
			self.label = None
		else:
			self.label = label
		return self

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
			self.door_x = float(reader.nextWord())
			self.door_y = float(reader.nextWord())
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
		self.x1 = float(reader.nextWord())
		self.y1 = float(reader.nextWord())
		self.x2 = float(reader.nextWord())
		self.y2 = float(reader.nextWord())
		return self

def parseWallChain(reader):
	walls = []
	n = int(reader.nextWord())
	x1 = float(reader.nextWord())
	y1 = float(reader.nextWord())
	for i in range(n):
		x2 = float(reader.nextWord())
		y2 = float(reader.nextWord())
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
		self.x = float(reader.nextWord())
		self.y = float(reader.nextWord())
		self.yaw = float(reader.nextWord())
		self.wp1 = int(reader.nextWord())
		self.wp2 = int(reader.nextWord())
		self.open = (True if reader.nextWord() == "open" else False)
		return self

###### OBJECTS ######

class ObjectInfo:
	def __init__(self):
		self.vals = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		self.rgb = [ 0, 0, 0 ]
		self.cats = []
		self.labels = []
		self.desc = ""

	def read_info(self, reader):
		self.obj_id = int(reader.nextWord())
		for i in range(0, 9):
			self.vals[i] = float(reader.nextWord())
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

