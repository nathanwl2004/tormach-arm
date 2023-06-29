import rclpy
from machinekit import hal

# Make sure the realtime runtime is started only after whole configuration
# is loaded and not sooner

logger = rclpy.logging.get_logger("ZA_hal_config_start_realtime")


def start_realtime():
    logger.info('Starting HAL threads')
    hal.start_threads()
    logger.info('HAL threads started')
