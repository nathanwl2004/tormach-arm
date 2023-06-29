from hw_device_mgr.mgr_ros_hal.mgr import ROSHALHWDeviceMgr
from hw_device_mgr.ethercat.device import EtherCATSimDevice
from hw_device_mgr.hal.device import HALPinSimDevice
from hw_device_mgr.lcec.device import LCECDevice, LCECSimDevice
from hw_device_mgr.devices.inovance_is620n import InovanceIS620N
from hw_device_mgr.devices.inovance_sv660 import InovanceSV660
from hw_device_mgr.devices.itegva_e7x import ITegvaE7820003ByteDevice
from hw_device_mgr.devices.bogus import BogusV1Servo


class ZADrives(LCECDevice):
    """Device category:  ZA drives + IO devices, both HW and sim."""

    category = "ZA_devices"


class ZAInovanceSV660(ZADrives, InovanceSV660):
    """ZA Inovance SV660 servo drive on LCEC."""

    name = "ZA_SV660"


class ZAInovanceIS620N(ZADrives, InovanceIS620N):
    """ZA Inovance IS620N servo drive on LCEC."""

    name = "ZA_IS620N"


class ZAITegvaE7820003ByteDevice(ZADrives, ITegvaE7820003ByteDevice):
    """ZA iTegva E7.820.003 16 Dig In/16 Mosfet Out Access Byte on LCEC."""

    name = "ZA_E7.820.003"


class ZAHWDeviceMgr(ROSHALHWDeviceMgr):
    """The ZA drive manager."""

    name = "ZA_device_mgr"
    device_base_class = ZADrives
    device_classes = (
        ZAInovanceSV660,
        ZAInovanceIS620N,
        ZAITegvaE7820003ByteDevice,
    )


class ZASimDrives(EtherCATSimDevice, HALPinSimDevice):
    """Device category:  ZA drives + IO devices, both HW and sim."""

    category = "ZA_devices"


class ZASimDrive(BogusV1Servo, LCECSimDevice, ZASimDrives):
    """ZA "Bogo Co." EtherCAT motor drive on sim bus."""

    name = "ZA_sim_drive"


class ZASimHWDeviceMgr(ROSHALHWDeviceMgr):
    """The ZA sim drive manager."""

    # Real ROS+HAL drive manager, but sim drives

    name = "ZA_sim_device_mgr"
    device_base_class = ZASimDrives
    device_classes = (ZASimDrive,)
