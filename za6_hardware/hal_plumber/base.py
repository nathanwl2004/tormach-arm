from enum import IntEnum
import rclpy
from hal_hw_interface.loadrt_local import hal

###########################################
# HALPlumberBase
#
# Base class for everything else; basically exists to provide a
# mechanism for adding HAL functions in the right order


class HALPlumberBase:
    class Prio(IntEnum):
        # Conventions for where to add update commands in thread
        LIST_HEAD = 0  # The head of the thread linked list
        DRIVE_READ_FB = 100  # Read fb from drive
        SAFETY_CHAIN = 300  # Check safety conditions
        FB_CHAIN = 400  # Process fb
        ROS_CONTROL = 500  # ros_control read(); update(); write()
        CMD_CHAIN = 700  # Process cmd
        DRIVE_WRITE_CMD = 900  # Write cmd to drive
        LIST_TAIL = 10000  # The tail of the thread linked list

    # A dict with prio:func_name mappings
    thread_functs = dict()

    def __init__(self, name, params):
        self.name = name
        self.logger = rclpy.logging.get_logger(self.name)
        self.process_params(params)

    param_names = (
        'hal_thread',
        'hardware_settings',
        'joint_limits',
        'sim_device_data_path',
        'sim_mode',
    )

    def process_params(self, params):
        self.params = params
        for key in self.param_names:
            assert key in params, f"params has no key '{key}'"
            setattr(self, key, params[key])

    def func_config(self, name, prio):
        if prio in self.thread_functs:
            raise RuntimeError(
                "Function '%s' already set with prio %d"
                % ((self.thread_functs[prio], prio))
            )
        self.thread_functs[prio] = name

    def add_funcs(self):
        # Call HAL addf for each function in correct thread order
        for prio in sorted(self.thread_functs.keys()):
            self.logger.info(f"Adding HAL function {self.thread_functs[prio]}")
            hal.addf(self.thread_functs[prio], self.thread_name)
