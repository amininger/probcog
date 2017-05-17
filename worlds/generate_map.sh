./parsemap.py $1
./createworld.py $1
cp $1.tagdb ../config/
cp $1.soar ../../rosie/agent/manage-world-state/world/maps/
