from pysoarlib import *

from mobilesim.lcmtypes import rosie_agent_command_t

# This is a simple connector that listens for a ROSIE_AGENT_COMMAND lcm message
# command_type == INTERRUPT:
#    will stop the agent
# command_type == CONTINUE:
#    will run the agent
class AgentCommandConnector(AgentConnector):
    def __init__(self, agent, lcm):
        super().__init__(agent)

        self.lcm = lcm
        self.lcm_handler = lambda channel, data: self.message_received(channel, data)
        self.lcm_subscriptions = []

    def connect(self):
        super().connect()
        self.lcm_subscriptions.append(self.lcm.subscribe("ROSIE_AGENT_COMMAND", self.lcm_handler))

    def disconnect(self):
        for sub in self.lcm_subscriptions:
            self.lcm.unsubscribe(sub)
        self.lcm_subscriptions = []
        super().disconnect()

    def message_received(self, channel, data):
        if channel == "ROSIE_AGENT_COMMAND":
            agent_command = rosie_agent_command_t.decode(data)
            if agent_command.command_type == rosie_agent_command_t.INTERRUPT:
                self.agent.stop()
            elif agent_command.command_type == rosie_agent_command_t.CONTINUE:
                self.agent.start()

