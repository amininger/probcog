#!/usr/bin/env python3

import sys

def pNum(n):
	return ("%.2f" % n).rjust(6)

def create_object_file(world_info, obj_info_filename):
	fout = open(obj_info_filename, 'w')

	for obj in world_info.objects:
		printObject(obj, fout)

	fout.close()

##### OBJECTS #####

def printObject(obj_info, fout):
	fout.write(str(obj_info.obj_id))
	for i in range(0, 3):
		fout.write(" " + str(obj_info.vals[i]))
	for i in range(6, 9):
		fout.write(" " + str(obj_info.vals[i]))
	fout.write(" " + str(len(obj_info.cats)))
	for i in range(len(obj_info.cats)):
		fout.write(" " + obj_info.cats[i] + " " + obj_info.labels[i])
	fout.write("\n")

