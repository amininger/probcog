import time
current_time_us = lambda: int(time.time() * 1000000)
import threading

from pysoarlib import *
from .SoarCommandParser import create_empty_control_law, parse_control_law
from mobilesim.lcmtypes import control_law_status_t

CONTROL_LAW_RATE = 10

class MobileSimActuationConnector(AgentConnector):
    def __init__(self, agent, lcm):
        AgentConnector.__init__(self, agent)

        self.lock = threading.Lock()
        self.lcm = lcm
        self.lcm_handler = lambda channel, data: self.message_received(channel, data)
        self.lcm_subscriptions = []

        self.add_output_command("do-control-law")
        self.add_output_command("stop")

        self.moving_status = "stopped"

        self.next_control_law_id = 1
        self.active_command = create_empty_control_law("RESTART")
        self.active_command.id = self.next_control_law_id
        self.next_control_law_id += 1

        self.active_command_id = None
        self.status_wme = SoarWME("status", "none")

    def get_moving_status(self):
        return self.moving_status
        
    # Start listening for LCM messages and start a thread to send current control law via LCM
    def connect(self):
        super().connect()
        self.lcm_subscriptions.append(self.lcm.subscribe("SOAR_COMMAND_STATUS.*", self.lcm_handler))
        self.kill_thread = False
        self.send_command_thread = threading.Thread(target=MobileSimActuationConnector.send_command_thread_fn, args=(self,))
        self.send_command_thread.start()

    # Stop listening for LCM messages and stop the send message thread
    def disconnect(self):
        super().disconnect()

        for sub in self.lcm_subscriptions:
            self.lcm.unsubscribe(sub)
        self.lcm_subscriptions = []

        if self.send_command_thread is not None:
            self.kill_thread = True
            self.send_command_thread.join()
            self.send_command_thread = None

    # When we receive an LCM message with a status, update it
    def message_received(self, channel, data):
        self.lock.acquire()
        if channel.startswith("SOAR_COMMAND_STATUS"):
            status = control_law_status_t.decode(data)
            if self.active_command is not None and self.active_command.id == status.id:
                self.status_wme.set_value(str(status.status).lower())
        self.lock.release()
    
    # Periodically re-send the currently active command
    def send_command_thread_fn(self):
        while not self.kill_thread:
            self.lock.acquire()
            if self.active_command is not None:
                self.active_command.utime = current_time_us()
                self.lcm.publish("SOAR_COMMAND_TX", self.active_command.encode())
            self.lock.release()
            time.sleep(1.0/CONTROL_LAW_RATE)

    # On the input phase, update the status wme
    #   and stop the command broadcasting if we got a terminal status
    def on_input_phase(self, input_link):
        self.lock.acquire()
        if self.active_command_id is not None:
            self.status_wme.update_wm(self.active_command_id)
            if self.status_wme.get_value() in [ "success", "failure", "unknown" ]:
                self.active_command_id = None
                self.active_command = None
                self.moving_status = "stopped"
        self.lock.release()

    def on_init_soar(self):
        self.lock.acquire()
        self.active_command = None
        self.active_command_id = None
        self.status_wme = SoarWME("status", "none")
        self.lock.release()


    #################################################
    #
    # HANDLING SOAR COMMANDS
    #
    ################################################

    def on_output_event(self, command_name, root_id):
        if command_name == "do-control-law":
            self.process_do_control_law(root_id)
        elif command_name == "stop":
            self.process_stop_command(root_id)

    def process_do_control_law(self, root_id):
        control_law = parse_control_law(root_id)
        if control_law is None:
            root_id.CreateStringWME("status", "error")
            root_id.CreateStringWME("error-type", "syntax-error")
            return

        self.lock.acquire()
        if self.active_command_id is not None:
            self.status_wme.set_value("interrupted")
            self.status_wme.update_wm(self.active_command_id)

        self.active_command = control_law
        self.active_command.id = self.next_control_law_id
        self.next_control_law_id += 1

        self.active_command_id = root_id
        self.status_wme = SoarWME("status", "sent")
        self.status_wme.update_wm(self.active_command_id)
        self.moving_status = "moving"
        self.lock.release()

    def process_stop_command(self, root_id):
        self.lock.acquire()
        if self.active_command_id is not None:
            self.status_wme.set_value("interrupted")
            self.status_wme.update_wm(self.active_command_id)

        self.active_command = create_empty_control_law("STOP")
        self.active_command.id = self.next_control_law_id
        self.next_control_law_id += 1

        self.active_command.id = root_id
        self.status_wme = SoarWME("status", "sent")
        self.status_wme.update_wm(self.active_command_id)
        self.moving_status = "stopped"
        self.lock.release()

