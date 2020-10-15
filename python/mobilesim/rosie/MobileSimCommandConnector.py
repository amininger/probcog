from pysoarlib import *

from rosie import CommandHandler
from mobilesim.lcmtypes import rosie_agent_command_t
from mobilesim.rosie.actuation import ControlLawUtil

# This is a simple connector that listens for a ROSIE_AGENT_COMMAND lcm message
# command_type == INTERRUPT:
#    will stop the agent
# command_type == CONTINUE:
#    will run the agent
class MobileSimCommandConnector(CommandHandler):
    def __init__(self, agent, lcm, print_handler=None):
        CommandHandler.__init__(self, agent, print_handler)

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

    def _handle_move_command(self, obj_id, x, y, z, wp_handle):
        cl_params = { 'object-id': obj_id, 'x': x, 'y': y, 'z': z }
        cond_test = ControlLawUtil.create_empty_condition_test("stabilized")
        control_law = ControlLawUtil.create_control_law("put-at-xyz", cl_params, cond_test)
        self.agent.get_connector('actuation').queue_command(control_law, self.callback)

    def _handle_place_command(self, obj_id, rel_handle, dest_id):
        cl_params = { 'object-id': obj_id, 'relation': rel_handle, 'destination-id': dest_id }
        cond_test = ControlLawUtil.create_empty_condition_test("stabilized")
        control_law = ControlLawUtil.create_control_law("put-on-object", cl_params, cond_test)
        self.agent.get_connector('actuation').queue_command(control_law, self.callback)

    def _handle_set_pred_command(self, obj_id, prop_handle, pred_handle):
        cl_params = { 'object-id': obj_id, 'property': prop_handle, 'value': pred_handle }
        cond_test = ControlLawUtil.create_empty_condition_test("stabilized")
        control_law = ControlLawUtil.create_control_law("change-state", cl_params, cond_test)
        self.agent.get_connector('actuation').queue_command(control_law, self.callback)

