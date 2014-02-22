#!/usr/bin/python

import sys, os

files = sys.argv[1:]

for ifile in files:
	ofile = ifile + '.tmp'
	fin = open(ifile, 'r')
	fout = open(ofile, 'w')
	for line in fin:
		fout.write(line)
		if (line.find('{') >= 0):
			fout.write("\t-1\n")
	fin.close()
	fout.close()
	os.remove(ifile)
	os.rename(ofile, ifile)

