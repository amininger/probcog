import sys
import os
import shutil

from parse_info_file import parse_info_file
from create_world import create_world
from create_tagdb import create_tagdb
from create_soar_map import create_soar_map
from create_object_file import create_object_file

# Arg 1: stem of the info file

if len(sys.argv) <= 1:
	print("Given a file stem and a matching info file in map_info")
	print("  Generates a set of world/map files for rosie")
	print("Example: given the stem of 'simple_office'")
	print("  Input File : map_info/simple_office.info")
	print("  Output File: rosie-project/probcog/worlds/simple_office.world")
	print("  Output File: rosie-project/probcog/config/simple_office.tagdb")
	print("  Output File: rosie-project/rosie/agent/_agent/manage-world-state/world/maps/simple_office.soar")
	sys.exit(0)

world_stem = sys.argv[1]

# Arg 2: (optional) the directory of the rosie-project 
#   (can get from $ROSIE_PROJ instead)

rosie_path = ""
if len(sys.argv) > 2:
	rosie_path = sys.argv[2]
elif "ROSIE_PROJ" in os.environ:
	rosie_path = os.environ["ROSIE_PROJ"]
else:
	print("Error: Requires rosie-project directory")
	print("  Either set the $ROSIE_PROJ environment variable")
	print("  or give the directory path as a second argument")
	sys.exit(0)

# Read the file
info_filename = "map_info/" + world_stem + ".info"

print("Parsing info file: " + info_filename)
world_info = parse_info_file(info_filename)
print("Success!\n")


# Write the world file
world_filename = rosie_path + "/probcog/worlds/" + world_stem + ".world"

print("Writing world file: " + world_filename)
create_world(world_info, "temp.world")
shutil.copyfile("temp.world", world_filename)
print("Success!\n")


# Write the tagdb file
tagdb_filename = rosie_path + "/probcog/config/" + world_stem + ".tagdb"

print("Writing tagdb file: " + tagdb_filename)
create_tagdb(world_info, "temp.tagdb")
shutil.copyfile("temp.tagdb", tagdb_filename)
print("Success!\n")


# Write the soar map file
map_filename = rosie_path + "/rosie/agent/_agent/manage-world-state/world/maps/" + world_stem + ".soar"

print("Writing soar map file: " + map_filename)
create_soar_map(world_info, "temp.soar")
shutil.copyfile("temp.soar", map_filename)
print("Success!\n")


# Write the object info file
obj_filename = rosie_path + "/probcog/worlds/objects/" + world_stem + ".info"

print("Writing object info file: " + obj_filename)
create_object_file(world_info, "temp.objinfo")
shutil.copyfile("temp.objinfo", obj_filename)
print("Success!\n")

