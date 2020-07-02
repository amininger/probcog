import sys

import Python_sml_ClientInterface as sml

from rosie import RosieAgent

from mobilesim.rosie import LCMConnector, AgentCommandConnector, MobileSimPerceptionConnector, MobileSimActuationConnector

class MobileSimAgent(RosieAgent):
    def __init__(self, config_filename=None, **kwargs):
        RosieAgent.__init__(self, config_filename=config_filename, verbose=False, **kwargs)

        lcm = LCMConnector(self)
        self.connectors["lcm"] = lcm
        self.connectors["actuation"] = MobileSimActuationConnector(self, lcm.lcm)
        self.connectors["perception"] = MobileSimPerceptionConnector(self, lcm.lcm)
        self.connectors["agent_cmd"] = AgentCommandConnector(self, lcm.lcm)

