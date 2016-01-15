#!/usr/bin/env python3[

import sys
from math import sqrt, atan2

class FileReader:
	def __init__(self, file):
		self.f = file
		self.splitline = None
		self.lineIndex = 0

	def nextWord(self):
		while True:
			if self.splitline != None and self.lineIndex < len(self.splitline):
				self.lineIndex = self.lineIndex + 1
				return self.splitline[self.lineIndex - 1]
			line = self.nextLine()
			if line == None:
				self.splitline = None
				return None
			else:
				self.splitline = line.split()
				self.lineIndex = 0

	def nextLine(self):
		# Read file until the end is reached or a non-empty line
		while True:
			line = self.f.readline()
			if len(line) == 0:
				return None
			if len(line.split()) > 0:
				return line

fin = open("test", 'r')
reader = FileReader(fin)

word = reader.nextWord()
while word != None:
	print("|" + word + "|")
	word = reader.nextWord()

fin.close()


