import sys
import os
import shutil

from parse_info_file import parse_info_file
from create_world import create_world
from create_tagdb import create_tagdb
from create_soar_map import create_soar_map
from create_object_file import create_object_file
from create_map_info import create_map_info
from create_internal_world import create_internal_world

# Arg 1: stem of the info file

if len(sys.argv) <= 1:
    print("python3 generate_files.py [info_name] [rosie_agent]")
    print("  info_name will parse the file at map_info/[info_name].info")
    print("  rosie_agent is optional and is the name of an agent in $ROSIE_HOME/test-agents")
    print("Will generate a set of world/map files for a rosie environment, both magicbot and internal")
    print("Example: given the stem of 'simple_office' and agent 'driver'")
    print("  Input File : map_info/simple_office.info")
    print("  Output File: $PROBCOG_HOME/worlds/simple_office.world")
    print("  Output File: $PROBCOG_HOME/worlds/objects/simple_office.info")
    print("  Output File: $PROBCOG_HOME/worlds/maps/simple_office.map")
    print("  Output File: $ROSIE_PROJ/rosie/agent/manage-world-state/waypoint-maps/simple_office.soar")
    print("  Output File: $ROSIE_PROJ/rosie/agent/manage-world-state/internal-worlds/simple_office.soar")
    print("  Output File: $ROSIE_PROJ/rosie/test-agents/driver/simple_office.obj_info")
    print("  Output File: $ROSIE_PROJ/rosie/test-agents/driver/simple_office.map")
    sys.exit(0)

world_stem = sys.argv[1]

# Arg 2: (optional) the name of the rosie test-agent to copy things to 

rosie_agent = None
if len(sys.argv) > 2:
    rosie_agent = sys.argv[2]

rosie_path = ""
if "ROSIE_PROJ" in os.environ:
    rosie_path = os.environ["ROSIE_PROJ"]
else:
    print("ERROR: Requires ROSIE_PROJ environment variable set")
    sys.exit(0)


probcog_home = ""
if "PROBCOG_HOME" in os.environ:
    probcog_home = os.environ["PROBCOG_HOME"]
else:
    print("ERROR: Requires PROBCOG_HOME environment variable set")
    sys.exit(0)


# Read the file
info_filename = "map_info/" + world_stem + ".info"

print("Parsing info file: " + info_filename)
try:
    world_info = parse_info_file(world_stem, info_filename)
except Exception as e:
    print(e)
    sys.exit(0)
print("Success!\n")


# Write the world file
world_filename = probcog_home + "/worlds/" + world_stem + ".world"

print("Writing world file: " + world_filename)
create_world(world_info, "temp.world")
shutil.copyfile("temp.world", world_filename)
print("Success!\n")


## Write the tagdb file
#tagdb_filename = rosie_path + "/probcog/config/" + world_stem + ".tagdb"
#
#print("Writing tagdb file: " + tagdb_filename)
#create_tagdb(world_info, "temp.tagdb")
#shutil.copyfile("temp.tagdb", tagdb_filename)
#print("Success!\n")


# Write the soar map file
map_folder = rosie_path + "/rosie/agent/manage-world-state/waypoint-maps/"
map_filename = map_folder + world_stem + ".soar"

print("Writing soar map file: " + map_filename)
create_soar_map(world_info, "temp.soar")
shutil.copyfile("temp.soar", map_filename)

print("Writing maps_source.soar file")
fout = open("maps_source.soar", 'w')
fout.write("source " + world_stem + ".soar")
fout.close()
shutil.copyfile("maps_source.soar", map_folder + "maps_source.soar")

print("Success!\n")


# Write the internal-world soar file
internal_filename = rosie_path + "/rosie/agent/manage-world-state/internal-worlds/" + world_stem + ".soar"

print("Writing internal world soar file: " + internal_filename)
create_internal_world(world_info, "temp2.soar")
shutil.copyfile("temp2.soar", internal_filename)


# Write the object info file
obj_filename = probcog_home + "/worlds/objects/" + world_stem + ".info"

print("Writing object info file: " + obj_filename)
create_object_file(world_info, "temp.objinfo")
shutil.copyfile("temp.objinfo", obj_filename)
if rosie_agent:
    obj_filename = rosie_path + "/rosie/test-agents/" + rosie_agent + "/" + world_stem + ".obj_info"
    print(" -> " + obj_filename)
    shutil.copyfile("temp.objinfo", obj_filename)
print("Success!\n")


# Write the map info file
map_filename = probcog_home + "/worlds/maps/" + world_stem + ".map"

print("Writing map info file: " + map_filename)
create_map_info(world_info, "temp.map")
shutil.copyfile("temp.map", map_filename)
if rosie_agent:
    map_filename = rosie_path + "/rosie/test-agents/" + rosie_agent + "/" + world_stem + ".map"
    print(" -> " + map_filename)
    shutil.copyfile("temp.map", map_filename)
print("Success!\n")

print("Success!\n")
