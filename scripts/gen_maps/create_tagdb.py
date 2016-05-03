#!/usr/bin/env python3

import sys

def create_tagdb(world_info, tagdb_filename):
	fout = open(tagdb_filename, 'w')

	# Start of file
	fout.write("classes {\n")

	tag_labels = []

	for wp in world_info.waypoints:
		printTagId(wp.tag_id, fout)

		if wp.label == None:
			continue

		# Add the tag to the list of labels
		found = False
		for l in tag_labels:
			if wp.label == l.label:
				l.tags.append(wp.tag_id)
				found = True
				break

		if not found:
			new_label = TagLabel(wp.label)
			new_label.tags = [wp.tag_id]
			tag_labels.append(new_label)
	
	for tag_label in tag_labels:
		printTagLabel(tag_label, fout)

	# End of file
	fout.write("}\n")
	fout.close()


def pNum(n):
	return ("%.2f" % n).rjust(6)

class TagLabel:
	def __init__(self, label):
		self.label = label
		self.tags = []

def printTagId(tag_id, fout):
	tag_id_str = str(tag_id)
	if len(tag_id_str) < 2:
		tag_id_str = "0" + tag_id_str
	fout.write("  c# {\n")
	fout.write("    labels = [\"wp" + tag_id_str + "\", \"\"];\n")
	fout.write("    ids = [" + str(tag_id) + "];\n")
	fout.write("    probs = [1.0, 0.0];\n")
	fout.write("    mean = 1.0;\n")
	fout.write("    stddev = 0.0;\n")
	fout.write("    minRange = 1.0;\n")
	fout.write("    maxRange = 2.0;\n")
	fout.write("  }\n")
	fout.write("\n")

def printTagLabel(tag_label, fout):
	fout.write("  c# {\n")
	fout.write("    labels = [\"" + tag_label.label + "\", \"\"];\n")
	fout.write("    ids = [" + str(tag_label.tags[0]))
	for other in tag_label.tags[1:]:
		fout.write(", " + str(other))
	fout.write("];\n")
	fout.write("    probs = [1.0, 0.0];\n")
	fout.write("    mean = 1.0;\n")
	fout.write("    stddev = 0.0;\n")
	fout.write("    minRange = 1.0;\n")
	fout.write("    maxRange = 2.0;\n")
	fout.write("  }\n")
	fout.write("\n")

