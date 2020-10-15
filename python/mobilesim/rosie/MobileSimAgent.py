import sys

import Python_sml_ClientInterface as sml

from rosie import RosieAgent

from mobilesim.rosie import LCMConnector, MobileSimCommandConnector, MobileSimPerceptionConnector, MobileSimActuationConnector

from string import digits
strip_digits = lambda s: s.translate(str.maketrans('', '', digits))

class MobileSimAgent(RosieAgent):
    def __init__(self, config_filename=None, **kwargs):
        RosieAgent.__init__(self, config_filename=config_filename, verbose=False, domain="magicbot", **kwargs)

        self.lcm_conn = LCMConnector(self)
        self.add_connector("lcm", self.lcm_conn)

        self.actuation = MobileSimActuationConnector(self, self.lcm_conn.lcm)
        self.add_connector("actuation", self.actuation)

        self.perception = MobileSimPerceptionConnector(self, self.lcm_conn.lcm)
        self.add_connector("perception", self.perception)

        self.command_handler = MobileSimCommandConnector(self, self.lcm_conn.lcm)
        self.add_connector("commands", self.command_handler)


    def connect(self):
        super().connect()
        if "script" in self.connectors and self.find_help == "custom":
            self.get_connector("script").set_find_helper(lambda msg: self.handle_find_request(msg))

    def handle_find_request(self, msg):
        obj_cat = next(w for w in msg.split() if w[-1] == ',')
        obj_cat = obj_cat.replace(',', '')
        obj = self.connectors["perception"].objects.get_object_by_cat(obj_cat)
        container = self.connectors["perception"].objects.get_object_container(obj)
        response = "Unknown."
        if container is not None:
            response = "The {} is in the {}.".format(obj.get_property("category"), container.get_property("category"))
            response = strip_digits(response)
        return response

