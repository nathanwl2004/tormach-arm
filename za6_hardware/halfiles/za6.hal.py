#!/usr/bin/env python

import rclpy
import os
from hal_plumber import HALPlumberSim, HALPlumberEC

# from hal_plumber.robot_io_pins import setup_pins
from hal_plumber.start_realtime import start_realtime

logger = rclpy.logging.get_logger("ZA_hal_config")

# `parameters` symbol is magically placed in global namespace
params = parameters  # noqa:  F821

# Main ZA configuration
if params["sim_mode"]:
    logger.info("Loading HALPlumberSim config")
    plumber_class = HALPlumberSim
else:
    logger.info("Loading HALPlumberEC config")
    plumber_class = HALPlumberEC

try:
    plumber_class(params).setup_hal()
except (RuntimeError, NameError) as e:
    os.system("halcmd show")
    raise e
logger.info("HALPlumber config loaded")

# robot IO pins
# setup_pins(params)

# start threads
logger.info("Running start_realtime")
start_realtime()
logger.info("HAL config complete")
