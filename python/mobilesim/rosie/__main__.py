from tkinter import *
import tkinter.font

import sys

from mobilesim.rosie import RosieGUI

if len(sys.argv) == 1:
	print("Need to specify rosie config file as argument")
else:
	root = Tk()
	rosie_gui = RosieGUI(sys.argv[1], master=root)
	root.protocol("WM_DELETE_WINDOW", rosie_gui.on_exit)
	root.mainloop()
