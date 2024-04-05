#!/usr/bin/env python3
import os
import sys
import importlib

import rospy

from ros_pangu_agent.srv import HandlePanguAgentAction, HandlePanguAgentActionResponse
from llm_tools.code_extraction import extract_code

preamble_code = """
from action_library.action_library import *

"""


def import_action_sequence(path):
    """Imports action_sequence method from a given path to a script."""
    module_name = "action_sequence_module"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return getattr(module, "action_sequence")


class Node:

    code_filename = os.path.join("/tmp", "action_sequence.py")

    def __init__(self):
        rospy.init_node("exec_action_sequence_node")
        self.srv = rospy.Service(
            "handle_pangu_agent_action",
            HandlePanguAgentAction,
            self.srv_callback,
        )

    def save_code(self, code_action):
        code = preamble_code + code_action
        rospy.loginfo(f"store code: {code}")
        with open(self.code_filename, "w") as f:
            # f.write(preamble_code)
            f.write(code)
        rospy.loginfo(f"finish storing")

    def srv_callback(self, req):

        # Setup
        message = "completed action sequence successfully"
        rospy.loginfo("recieved request to execute an action sequence")
        # Extract and save code
        try:
            code = extract_code(req.action)
            self.save_code(code)
        except Exception as e:
            message = f"failed to extract/save code: {e}"
            rospy.logwarn(message)
            return HandlePanguAgentActionResponse(response=message, reward=0.0, success = False)

        # Import action sequence function
        # action_sequence = import_action_sequence(self.code_filename)
        try:
            action_sequence = import_action_sequence(self.code_filename)
        except Exception as e:
            message = f"failed to import function: {e}"
            rospy.logwarn(message)
            return HandlePanguAgentActionResponse(response=message, reward=0.0, success = False)

        # Execute action sequence
        try:
            action_sequence()
        except Exception as e:
            message = f"failed to execute action sequence: {e}"
            rospy.logwarn(message)
            return HandlePanguAgentActionResponse(response=message, reward=0.0, success = False)

        return HandlePanguAgentActionResponse(response=message, reward=1.0, success = True)

    def spin(Self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == "__main__":
    main()
