# Copyright (c) 2023 Tormach, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
