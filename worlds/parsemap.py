#!/usr/bin/env python3

import sys
from math import sqrt, atan2

def pNum(n):
	return ("%.2f" % n).rjust(6)

def printRobot(x, y, fout):
	fout.write("\"probcog.sim.SimRobot\"\n")
	fout.write("{\n")
	fout.write("  # Robot ID\n")
	fout.write("  6\n")
	fout.write("  # XYZRPY Truth\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(x), "y": pNum(-y), "z": pNum(0.5), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(0.0) })
	fout.write("  # XYZRPY Odometry\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(0.0), "y": pNum(0.0), "z": pNum(0.0), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(0.0) })
	fout.write("}\n")

def printWall(x1, y1, x2, y2, fout):
	dx = x2-x1
	dy = y2-y1
	l = sqrt(dx*dx + dy*dy)
	yaw = atan2(dy, dx)
	cx = (x1+x2)/2
	cy = (y1+y2)/2
	fout.write("\"april.sim.SimBox\"\n")
	fout.write("{\n")
	fout.write("  # XYZRPY\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(cx), "y": pNum(-cy), "z": pNum(0.5), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(-yaw) })
	fout.write("  # dimensions LWH\n")
	fout.write("  vec 3\n")
	fout.write("  %(l)s %(w)s %(h)s\n" % \
			{ "l": pNum(l), "w": pNum(0.1), "h": pNum(1.0) })
	fout.write("  # color RGBA\n")
	fout.write("  vec 4\n")
	fout.write("  %(r)s %(g)s %(b)s %(a)s\n" % \
			{ "r": pNum(0.2), "g": pNum(0.2), "b": pNum(0.2), "a": pNum(1.0)})
	fout.write("}\n")

def printTag(tagId, x, y, fout):
	fout.write("\"probcog.sim.SimAprilTag\"\n")
	fout.write("{\n")
	fout.write("  # XYZRPY\n")
	fout.write("  vec 6\n")
	fout.write("  %(x)s %(y)s %(z)s %(r)s %(p)s %(yaw)s\n" % \
			{ "x": pNum(x), "y": pNum(-y), "z": pNum(0.5), \
			  "r": pNum(0.0), "p": pNum(0.0), "yaw": pNum(0.0) })
	fout.write("  # Tag Id\n")
	fout.write("  " + str(tagId) + "\n")
	fout.write("}\n")

if len(sys.argv) == 1:
	print("No file specified")
	sys.exit(0)

stem = sys.argv[1]
mapfile = stem + ".map"
worldfile = stem + ".world"
infofile = stem + ".info"

fin = open(mapfile, 'r')

SIZE = 2.5

dims = fin.readline().split()
rows = int(dims[0])+2
cols = int(dims[1])+2

robotPos = fin.readline().split()
robotX = (int(robotPos[1]) + .5)*SIZE
robotY = (int(robotPos[0]) + .5)*SIZE

grid = [[' ' for c in range(cols)] for r in range(rows)]

for r in range(1, rows-1):
	line = fin.readline()
	s = "|"
	for c in range(1, cols-1):
		if len(line) - 1 > c - 1:
			grid[r][c] = line[c-1]

# Finished reading the file
fin.close()

# Start writing the world file
fout = open(worldfile, 'w')
printRobot(robotX, robotY, fout)

# Write the horizontal walls
for r in range(0, rows-1):
	lx = 1 * SIZE
	current_wall = False
	for c in range(0, cols):
		if (grid[r][c] != ' ' and grid[r+1][c] == ' ') or \
		   (grid[r][c] == ' ' and grid[r+1][c] != ' '):
			if not current_wall:
				current_wall = True
				lx = c * SIZE
		else:
			if current_wall:
				current_wall = False
				rx = c*SIZE
				printWall(lx, (r+1)*SIZE, rx, (r+1)*SIZE, fout)

# Write the vertical walls
for c in range(0, cols-1):
	ty = 1 * SIZE
	current_wall = False
	for r in range(0, rows):
		if (grid[r][c] != ' ' and grid[r][c+1] == ' ') or \
		   (grid[r][c] == ' ' and grid[r][c+1] != ' '):
			if not current_wall:
				current_wall = True
				ty = r * SIZE
		else:
			if current_wall:
				current_wall = False
				by = r*SIZE
				printWall((c+1)*SIZE, ty, (c+1)*SIZE, by, fout)

# Write the tags
tagNum = 0
nodes = []
for r in range(1, rows-1):
	for c in range(1, cols-1):
		if grid[r][c] != ' ' and grid[r][c] != 'X':
			tagNum = tagNum + 1
			node = { "id": str(tagNum), "col": c, "row": r, \
					"x": (c+.5)*SIZE, "y": (r+.5)*SIZE, "label": "" }
			if grid[r][c] == 'D':
				node["label"] = "door"
			elif grid[r][c] == 'I':
				node["label"] = "intersection"

			printTag(node["id"], node["x"], node["y"], fout)
			nodes.append(node)

# Finished writing the world file
fout.close()

# Start writing the info file
fout = open(infofile, 'w')

# Write the nodes
fout.write("nodes:\n")
for node in nodes:
	fout.write("%(id)2s %(x)s %(y)s %(c)s\n" % \
			{ "id": node["id"], "x": pNum(node["x"]), \
			  "y": pNum(-node["y"]), "c": node["label"] })


# Write the edges
fout.write("edges:\n")

def blocked(row, col, dr, dc, dist):
	for i in range(1, dist+1):
		if grid[row + i*dr][col + i*dc] == ' ':
			return True
	return False

for node in nodes:
	# NESW
	row = node["row"]
	col = node["col"]
	drs = [-1, 0, 1, 0]
	dcs = [0, 1, 0, -1]
	closest = [None, None, None, None]
	dists = [rows, cols, rows, cols]
	for other in nodes:
		dc = other["col"] - col
		dr = other["row"] - row
		# north
		#if dc == 0 and dr < 0 and -dr < dists[0]:
		#	closest[0] = other["id"]
		#	dists[0] = -dr
		# east
		if dr == 0 and dc > 0 and dc < dists[1]:
			closest[1] = other["id"]
			dists[1] = dc
		# south
		if dc == 0 and dr > 0 and dr < dists[2]:
			closest[2] = other["id"]
			dists[2] = dr
		# west 
		#if dr == 0 and dc < 0 and -dc < dists[3]:
		#	closest[3] = other["id"]
		#	dists[3] = -dc
	for d in range(4):
		other = closest[d]
		if other != None and not blocked(row, col, drs[d], dcs[d], dists[d]):
			fout.write(node["id"] + " " + other + "\n")

fout.close()



