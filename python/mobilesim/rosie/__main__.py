from tkinter import *
import tkinter.font

import sys

from rosie import RosieGUI
from mobilesim.rosie import MobileSimAgent

def agent_factory(config_file):
    return MobileSimAgent(config_file=config_file)

if len(sys.argv) == 1:
    print("Need to specify rosie config file as argument")
else:
    root = Tk()
    rosie_gui = RosieGUI(lambda: MobileSimAgent(config_filename=sys.argv[1]), master=root)
    root.protocol("WM_DELETE_WINDOW", rosie_gui.on_exit)
    root.mainloop()
