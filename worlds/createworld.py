fin = open("temp", 'r')
fout = open("world.soar", 'w')

fout.write("sp {topstate*elaborate*map\n")
fout.write("   (state <s> ^superstate nil)\n")
fout.write("-->\n")
fout.write("   (<s> ^map <map>)\n")
fout.write("   (<map> ^waypoints <wps>)\n")

for line in fin:
	data = line.split()
	padnum = data[0]
	if len(padnum) == 1:
		padnum = "0" + padnum
	wpname = "wp" + padnum
	wpid = "<" + wpname + ">"
	fout.write("   (<wps> ^waypoint " + wpid + ")\n")
	fout.write("   (" + wpid + " ^id " + data[0] + " ^x " + data[1] + " ^y " + data[2] + ")\n")
	for i in range(3, len(data)):
		if len(data[i]) == 1:
			data[i] = "0" + data[i]
		edgeid = "<e" + padnum + data[i] + ">"
		fout.write("    (" + wpid + " ^edge " + edgeid + ")\n")
		fout.write("    (" + edgeid + " ^start " + wpid + " ^end <wp" + data[i] + ">)\n")
	fout.write("\n")

fout.write("}\n")

fin.close()
fout.close()

