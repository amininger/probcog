#!/usr/bin/env python3

def create_soar_map(world_info, map_filename):
	nodes = {}
	edges = []

	for region in world_info.regions:
		node = Node(region)
		nodes[node.id_num] = node
	
	for edge in world_info.edges:
		edges.append(Edge(edge, True))
		edges.append(Edge(edge, False))

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
	def __init__(self, region):
		self.x = str(region.x)
		self.y = str(region.y)

		self.id_num = region.tag_id
		self.id_str = region.soar_id

		self.name = "wp" + self.id_str
		self.var = "<" + self.name + ">"
		self.classification = region.label
	
	def print(self, out):
		out.write('  (%(var)s ^handle %(hand)s ^handle-int %(id_num)d ^x %(x)s ^y %(y)s ^map <building>)\n' % \
				{ "var": self.var, "hand": self.name, "id_num": self.id_num, "x": self.x, "y": self.y })


class Edge:
	def __init__(self, edge, forward):
		if forward:
			self.start = edge.start_wp
			self.end = edge.end_wp
			self.side = 1
		else:
			self.start = edge.end_wp
			self.end = edge.start_wp
			self.side = -1
		self.has_door = edge.has_door
		if edge.has_door:
			self.door_x = str(edge.door_x)
			self.door_y = str(edge.door_y)
			self.door_rot = str(edge.door_rot)
	
	def print(self, nodes, fout):
		start = nodes[self.start]
		end = nodes[self.end]
		self.var = "<e" + start.id_str + end.id_str + ">"

		fout.write('   (%(start)s ^edge %(edge)s)\n' % \
				{ "start": start.var, "edge": self.var })
		fout.write('    (%(edge)s ^start %(start)s ^end %(end)s ^wall-side %(side)s\n' % \
				{ "edge": self.var, "start": start.var, "end": end.var, "side": self.side})
		if self.has_door:
			fout.write('         ^doorway true ^dx %(x)s ^dy %(y)s ^dr %(r)s)\n' % \
					{ "x": self.door_x, "y": self.door_y, "r": self.door_rot })
		else:
			fout.write('         ^doorway false)\n')
				
