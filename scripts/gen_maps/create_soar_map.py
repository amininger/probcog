#!/usr/bin/env python3

def create_soar_map(world_info, map_filename):
	nodes = {}
	edges = []

	for wp in world_info.waypoints:
		node = Node(wp)
		nodes[node.id_num] = node
	
	for edge in world_info.edges:
		edges.append(Edge(edge.start_wp, edge.end_wp, 1))
		edges.append(Edge(edge.end_wp, edge.start_wp, -1))

	fout = open(map_filename, 'w')

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
			if edge.start == node.id_num:
				edge.print(nodes, fout)
		fout.write("\n")

	fout.write("}\n")

	fout.close()

class Node:
	def __init__(self, wp_info):
		self.x = wp_info.x
		self.y = wp_info.y

		self.id_num = wp_info.tag_id
		self.id_str = str(self.id_num)

		# Note: this assumes all ids are less than 100 (and thus 2 characters)
		#   Increase this to accomodate more node ids
		while len(self.id_str) < 2:
			self.id_str = "0" + self.id_str

		self.name = "wp" + self.id_str
		self.var = "<" + self.name + ">"
		self.classification = wp_info.label
	
	def print(self, out):
		out.write('  (%(var)s ^handle %(hand)s ^handle-int %(id_num)d ^x %(x)s ^y %(y)s ^map <building>)\n' % \
				{ "var": self.var, "hand": self.name, "id_num": self.id_num, "x": self.x, "y": self.y })


class Edge:
	def __init__(self, start, end, side):
		self.start = start
		self.end = end
		self.side = side
	
	def print(self, nodes, fout):
		start = nodes[self.start]
		end = nodes[self.end]
		self.var = "<e" + start.id_str + end.id_str + ">"

		fout.write('   (%(start)s ^edge %(edge)s)\n' % \
				{ "start": start.var, "edge": self.var })
		fout.write('    (%(edge)s ^start %(start)s ^end %(end)s ^wall-side %(side)s)\n' % \
				{ "edge": self.var, "start": start.var, "end": end.var, "side": self.side})

