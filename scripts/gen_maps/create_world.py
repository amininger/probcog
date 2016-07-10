#!/usr/bin/env python3

import sys

def pNum(n):
	return ("%.2f" % n).rjust(6)

def create_world(world_info, world_filename):
	fout = open(world_filename, 'w')

	printRobot(world_info.robot, fout)

	for obj in world_info.objects:
		printObject(obj, fout)

	for wall in world_info.walls:
		printWall(wall, fout)

	for door in world_info.doors:
		printDoor(door, fout)

	for region in world_info.regions:
		printRegion(region, fout)

	fout.close()

###### ROBOT ######

def printRobot(robot_info, fout):
	fout.write("\"probcog.sim.SimRobot\"\n")
	fout.write("{\n")
	fout.write("  # Robot ID\n")
	fout.write("  6\n")
	fout.write("  # XYZRPY Truth\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(robot_info.x), "y": pNum(robot_info.y), "z": pNum(0.1), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(0.0) })
	fout.write("  # XYZRPY Odometry\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(0.0), "y": pNum(0.0), "z": pNum(0.0), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(0.0) })
	fout.write("}\n")

##### OBJECTS #####

def printObject(obj_info, fout):
	fout.write("\"probcog.sim.SimBoxObject\"\n")
	fout.write("{\n")
	fout.write("  # Object id\n")
	fout.write("  " + str(obj_info.obj_id) + "\n")
	fout.write("  # Object Description\n")
	fout.write("  " + str(obj_info.desc) + "\n")
	fout.write("  # Object xyzrpy\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(roll)s %(pitch)s %(yaw)s\n" % \
			{ "x": pNum(obj_info.vals[0]), "y": pNum(obj_info.vals[1]), "z": pNum(obj_info.vals[2]), \
			  "roll": pNum(obj_info.vals[3]), "pitch": pNum(obj_info.vals[4]), "yaw": pNum(obj_info.vals[5]) })
	fout.write("  # Length xyz\n")
	fout.write("  vec 3\n")
	fout.write("  %(x)s %(y)s %(z)s\n" % \
			{ "x": pNum(obj_info.vals[6]), "y": pNum(obj_info.vals[7]), "z": pNum(obj_info.vals[8]) })
	fout.write("  # Color rgb (int 0-255)\n")
	fout.write("  vec 3\n")
	fout.write("  %(r)d %(g)d %(b)d\n" % \
			{ "r": obj_info.rgb[0], "g": obj_info.rgb[1], "b": obj_info.rgb[2] })
	#fout.write("  # Labels \n")
	#fout.write("  " + str(len(obj_info.cats)) + "\n")
	#for i in range(len(obj_info.cats)):
	#	fout.write("  " + obj_info.cats[i] + "=" + obj_info.labels[i] + "\n")
	fout.write("}\n")

##### WALLS ######

def printWall(wall_info, fout):
	fout.write("\"probcog.sim.SimRoomWall\"\n")
	fout.write("{\n")
	fout.write("  # End Point 1\n")
	fout.write("  vec 2\n")
	fout.write("  %(x1)s %(y1)s\n" % \
			{ "x1": pNum(wall_info.x1), "y1": pNum(wall_info.y1) })
	fout.write("  # End Point 2\n")
	fout.write("  vec 2\n")
	fout.write("  %(x2)s %(y2)s\n" % \
			{ "x2": pNum(wall_info.x2), "y2": pNum(wall_info.y2) })
	fout.write("}\n")

##### REGIONS #####

def printRegion(region, fout):
	fout.write("\"probcog.sim.SimRegion\"\n")
	fout.write("{\n")
	fout.write("  # Region ID\n")
	fout.write("  " + str(region.tag_id) + "\n")
	fout.write("  # Tag Position\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(roll)s %(pitch)s %(yaw)s\n" % \
			{ "x": pNum(region.x), "y": pNum(region.y), "z": pNum(0.01), \
			  "roll": pNum(0), "pitch": pNum(0), "yaw": pNum(region.rot) })
	fout.write("  # Width (dx) and Length (dy)\n")
	fout.write("  " + str(region.width) + "\n")
	fout.write("  " + str(region.length) + "\n")
	fout.write("}\n")

##### DOOR #####

def printDoor(door_info, fout):
	fout.write("\"probcog.sim.SimDoor\"\n")
	fout.write("{\n")
	fout.write("  # Door xyzrpy\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(roll)s %(pitch)s %(yaw)s\n" % \
			{ "x": pNum(door_info.x), "y": pNum(door_info.y), "z": pNum(0), \
			  "roll": pNum(0), "pitch": pNum(0), "yaw": pNum(door_info.yaw) })
	fout.write("  # Connected Waypoints \n")
	fout.write("  " + str(door_info.wp1) + "\n")
	fout.write("  " + str(door_info.wp2) + "\n")
	fout.write("  # Open or closed\n")
	fout.write("  " + ("open" if door_info.open else "closed") + "\n")
	fout.write("}\n")

