#!/usr/bin/env python3

import sys

class Node:
	def __init__(self, line):
		data = line.split()

		self.pnum = data[0]
		if len(self.pnum) == 1:
			self.pnum = "0" + self.pnum
		self.x = data[1]
		self.y = data[2]

		self.num = int(self.pnum)
		self.name = "wp" + self.pnum
		self.var = "<" + self.name + ">"
	
	def print(self, out):
		fout.write('  (%(var)s ^handle %(hand)s ^handle-int %(num)d ^x %(x)s ^y %(y)s ^map <building>)\n' % \
				{ "var": self.var, "hand": self.name, "num": self.num, "x": self.x, "y": self.y })


class Edge:
	def __init__(self, start, end):
		self.start = start
		self.end = end
	
	def print(self, out):
		start = nodes[self.start]
		end = nodes[self.end]
		self.var = "<e" + start.pnum + end.pnum + ">"

		fout.write('   (%(start)s ^edge %(edge)s)\n' % \
				{ "start": start.var, "edge": self.var })
		fout.write('    (%(edge)s ^start %(start)s ^end %(end)s)\n' % \
				{ "edge": self.var, "start": start.var, "end": end.var })

if len(sys.argv) == 1:
	print("No file specified")
	sys.exit(0)

ifile = sys.argv[1] + ".info"
ofile = sys.argv[1] + ".soar"
fin = open(ifile, 'r')

NODES = 1
EDGES = 2

mode = NODES

nodes = {}
edges = []

for line in fin:
	if line.strip() == "":
		continue
	elif line.lower().startswith("node"):
		mode = NODES
	elif line.lower().startswith("edge"):
		mode = EDGES
	elif mode == NODES:
		node = Node(line)
		nodes[node.num] = node
	elif mode == EDGES:
		data = line.split()
		edges.append(Edge(int(data[0]), int(data[1])))
		edges.append(Edge(int(data[1]), int(data[0])))

fin.close()


fout = open(ofile, 'w')

fout.write("sp {topstate*elaborate*map\n")
fout.write("   (state <s> ^superstate nil)\n")
fout.write("-->\n")
fout.write("   (<s> ^maps <maps>)\n")
fout.write("   (<maps> ^map <building> <world>)\n")
fout.write("\n")
fout.write("   (<world> ^handle world-map ^waypoint <bwp01>)\n")
fout.write("   (<bwp01> ^handle bwp01 ^x 0 ^y 0 ^map <world> ^sub-map <building>)\n")
fout.write("\n")
fout.write("   ### BUILDING ###\n")
fout.write("\n")
fout.write("   (<building> ^handle bmap1 ^super-waypoint <bwp01>")
i = 0
for node_id in sorted(nodes):
	if i % 5 == 0:
		fout.write("\n       ^waypoint ")
	fout.write(nodes[node_id].var + " ")
	i = i + 1

fout.write(")\n")
fout.write("\n")
fout.write("   ### WAYPOINTS ###\n")
fout.write("\n")

for node_id in sorted(nodes):
	node = nodes[node_id]
	node.print(fout)
	for edge in edges:
		if edge.start == node.num:
			edge.print(fout)
	fout.write("\n")

fout.write("}\n")

fout.close()
