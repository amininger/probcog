import sys

from threading import Thread
import time

import Python_sml_ClientInterface as sml

from pysoarlib import SoarAgent, LanguageConnector

from mobilesim.rosie.perception import MobileSimPerceptionConnector
from mobilesim.rosie.actuation  import MobileSimActuationConnector

class RosieAgent(SoarAgent):
	def __init__(self, config_filename=None, **kwargs):
		SoarAgent.__init__(self, config_filename=config_filename, verbose=False, **kwargs)

		self.connectors["language"] = LanguageConnector(self)
		self.connectors["actuation"] = MobileSimActuationConnector(self)
		self.connectors["perception"] = MobileSimPerceptionConnector(self)
	
	def get_actuation(self):
		return self.connectors["actuation"]

	def get_perception(self):
		return self.connectors["perception"]

