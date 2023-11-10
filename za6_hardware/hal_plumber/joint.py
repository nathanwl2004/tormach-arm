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

from hal_hw_interface.loadrt_local import rtapi, hal
from .base import HALPlumberBase


###########################################
# HALJointPlumber and subclasses
#
# This does the per-joint configuration; subclass for specific drives


class HALJointPlumber(HALPlumberBase):
    # ========== Feedback chain:
    #
    # - The drive_fb signal input comes from drive subclasses
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # *_drive_fb            -> *_scale_pos_fb             -> *_ros_pos_fb
    # *_ros_pos_fb          -> hal_hw_interface.*.pos_fb
    #
    #
    # ========== Command chain:
    #
    # - The ros_pos_cmd signal output goes to drive subclasses
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # (N/A)                    hal_hw_interface.*.pos_cmd -> *_ros_pos_cmd
    # *_ros_pos_cmd         -> *_jlimits                  -> *_jlimits-out
    # *_jlimits-out         -> *_scale_pos_cmd            -> *_drive_cmd
    #
    #
    # ========== Reset:
    #
    # - The hw_device_mgr sets the reset signal high from just before
    #   enabling the drives to just after.
    #
    # - The reset signal connects to the hal_hw_interface reset pin to
    #   set command to feedback.
    #
    # - The reset signal connects to the jlimits.load pin to set
    #   output to input.
    #
    # ========== Safety input chain
    #
    # signal                -> comp/pin                  -> signal
    # ------                   --------                     ------
    # (TBD)
    #

    #########################
    # Convenience functions to save repeatedly typing and reading
    # `"joint%i_foo" % self.num`
    def jname(self, suffix):
        return "joint%i_%s" % (self.num, suffix)

    def newinst(self, comp_name, suffix, *args, **kwargs):
        return rtapi.newinst(comp_name, self.jname(suffix), *args, **kwargs)

    def newsig(self, suffix, haltype):
        return hal.newsig(self.jname(suffix), haltype)

    def pin(self, suffix):
        return hal.Pin(self.jname(suffix))

    def link_pin(self, from_suffix, to_suffix):
        self.pin(from_suffix).link(self.jname(to_suffix))

    def signal(self, suffix):
        return hal.Signal(self.jname(suffix))

    def link_global_signal(self, signal, *pin_suffixes):
        if isinstance(signal, str):
            signal = hal.Signal(signal)
        for s in pin_suffixes:
            signal.link(self.jname(s))

    def link_signal(self, sig_suffix, *pin_suffixes):
        self.link_global_signal(self.signal(sig_suffix), *pin_suffixes)

    def set_pin(self, suffix, value):
        self.pin(suffix).set(value)

    def set_signal(self, suffix, value):
        self.signal(suffix).set(value)

    def get_pin(self, suffix):
        return self.pin(suffix).get()

    def func_config(self, base_name, base_prio):
        name = self.jname(base_name)
        prio = base_prio + (0.1 * self.num)
        super().func_config(name, prio)

    #########################
    # Set up HAL for a joint
    def __init__(self, name, params):
        self.idx = params['hardware_settings'][name]['slavenum']
        self.device_config = params["device_configs"][self.idx]
        params = params.copy()  # Destructive operations coming up
        params.update(  # Flatten out per-joint dicts
            hardware_settings=params['hardware_settings'][name],
            joint_limits=params['joint_limits'][name],
        )
        # IS620N drives don't have STO function (SV660N and sim do)
        self.have_sto = self.device_config.model_id != (0x00100000, 0x000C0108)
        super().__init__(name, params)
        self.logger.info(f"Initialized joint {self.name}")

    joint_param_names = (
        'acc_i',
        'acc_p',
        'cmd_bandwidth',
        'damping',
        'fb_bandwidth',
        'friction',
        'gear_ratio',
        'inertia',
        'load',
        'max_acc',
        'max_pos',
        'max_torque',
        'max_vel',
        'min_pos',
        'num',
        'online_estimation',
        'pos_g',
        'pos_p',
        'sim_startup_position',
        'slavenum',
        'torque_boost',
        'vel_i',
        'vel_p',
    )

    def process_params(self, params):
        super().process_params(params)
        # Add hardware_settings keys as well
        for key in self.joint_param_names:
            assert key in self.hardware_settings
            assert not hasattr(self, key)
            setattr(self, key, self.hardware_settings[key])

    def setup_hal(self):
        # This represents the high-level work of per-joint HAL setup
        # of feedback and command pipeline comps, signals, settings
        #
        # Set up feedback and command pipeline comps, signals, settings
        self.init_plumbing()
        # - Connect drive-specific config
        self.connect_drive()
        self.init_max_vel_safety()
        # - Per-joint misc stuff
        self.setup_jlimits()
        self.connect_hal_ros_control()
        # - Drive-specific final operations, optionally implemented in
        #   subclasses
        self.finish_config()

    cw_bits = [
        'switch_on',
        'enable_voltage',
        'quick_stop',
        'enable_operation',
        'mode_1',
        'mode_2',
        'mode_3',
        'fault_reset',
        'halt',
        # Bits 9-15 reserved
    ]

    def init_plumbing(self):
        # joint limits
        self.newinst("limit3v3", "jlimits")
        # Safety input
        self.newsig("max_vel_safety", hal.HAL_FLOAT)

        self.newsig('ros_pos_cmd', hal.HAL_FLOAT)
        self.newsig('jlimits_pos_cmd', hal.HAL_FLOAT)
        self.newsig('ros_vel_cmd', hal.HAL_FLOAT)
        self.newsig('ros_acc_cmd', hal.HAL_FLOAT)

        self.newsig('pos_cmd', hal.HAL_FLOAT)
        self.newsig('pos_fb', hal.HAL_FLOAT)
        self.newsig('pos_err', hal.HAL_FLOAT)
        self.newsig('vel_cmd', hal.HAL_FLOAT)
        self.newsig('vel_fb', hal.HAL_FLOAT)
        self.newsig('acc_cmd', hal.HAL_FLOAT)
        self.newsig('acc_fb', hal.HAL_FLOAT)
        self.newsig('torque_cmd', hal.HAL_FLOAT)
        self.newsig('torque_fb', hal.HAL_FLOAT)

        # Drive control:  Link hw_device_mgr pins to signals
        # - raw_control_word sig:  raw control_word
        cwr_sig = self.newsig('raw_control_word', hal.HAL_U32)
        cwr_sig.link(f"hw_device_mgr.0.{self.slavenum}.0.control_word")
        cwr_sig.link(f"drive_safety.raw-control-word{self.slavenum}")
        # - control_word sig:  raw control_word w/emergency stop override
        cw_sig = self.newsig('control_word', hal.HAL_U32)
        cw_sig.link(hal.Pin(f'drive_safety.control-word{self.slavenum}'))
        # - device mgr drive-side interfaces
        for name, htype in (
            ('status_word', hal.HAL_U32),
            ('control_mode', hal.HAL_S32),
            ('control_mode_fb', hal.HAL_S32),
            ('error_code', hal.HAL_U32),
            ('online', hal.HAL_BIT),
            ('oper', hal.HAL_BIT),
            *((('sto', hal.HAL_BIT),) if self.have_sto else tuple()),
        ):
            sig = self.newsig(name, htype)
            sig.link(f"hw_device_mgr.0.{self.slavenum}.0.{name}")
        # - link device mgr control-side interfaces to drive_state comp
        for name, htype in (
            ('home_request', hal.HAL_BIT),
            ('home_success', hal.HAL_BIT),
            ('home_error', hal.HAL_BIT),
        ):
            sig = self.newsig(name, htype)
            sig.link(f"hw_device_mgr.d0.{self.slavenum}.0.{name}")
            sig.link(f"drive_state.{self.slavenum}.{name}")
        # - link position + velocity fb + cmd to drive_state comp
        for name in ('pos_fb', 'vel_fb', 'pos_cmd'):
            self.signal(name).link(f"drive_state.{self.slavenum}.{name}")
        # - status_word:  link to drive_safety
        sw_sig = self.signal('status_word')
        sw_sig.link(f'drive_safety.status-word{self.slavenum}')
        # - control_mode (cmd):  link to common control_mode signal
        cm_sig = hal.Signal('control_mode')
        cm_sig.link(f"hw_device_mgr.d0.{self.slavenum}.0.control_mode")

    def init_max_vel_safety(self):
        # Set up joint max_vel_safety signal from the configured joint
        # max_vel * global max_vel_safety_scale signal.
        # max_vel_safety_scale is normally 1.0, and changes to
        # something closer to 0.0 when the safety_input goes low.
        self.newinst("mult2v2", "max_vel_safety")

        self.link_global_signal('max_vel_safety_scale', "max_vel_safety.in0")
        self.set_pin("max_vel_safety.in1", self.max_vel)
        self.link_signal("max_vel_safety", "max_vel_safety.out")

        self.func_config("max_vel_safety.funct", self.Prio.SAFETY_CHAIN + 20)

    def setup_jlimits(self):
        # Set max acceleration and velocity
        self.set_pin("jlimits.maxa", self.max_acc)
        self.set_pin("jlimits.maxv", self.max_vel)
        self.set_pin("jlimits.max", self.max_pos)
        self.set_pin("jlimits.min", self.min_pos)

        # Schedule update function
        self.func_config("jlimits.funct", self.Prio.CMD_CHAIN + 30)

        # ROS joint (cmd) position to jlimits
        self.signal('ros_pos_cmd').link(self.pin("jlimits.in"))
        # connect jlimits output
        self.signal('jlimits_pos_cmd').link(self.pin("jlimits.out"))
        # connect load pin
        hal.Signal("load").link(self.pin("jlimits.load"))

    def connect_drive(self):
        # Override in classes
        raise RuntimeError("Subclasses must implement connect_drive() method")

    def connect_hal_ros_control(self):
        # ROS commanded position
        self.signal('ros_pos_cmd').link(
            "hal_hw_interface.joint_%i.position_cmd" % self.num
        )

        # Joint feedback to ROS
        self.signal('pos_fb').link(
            f'hal_hw_interface.joint_{self.num}.position_fb'
        )
        self.signal('vel_fb').link(
            f'hal_hw_interface.joint_{self.num}.velocity_fb'
        )

        self.signal('torque_fb').link(
            f'hal_hw_interface.joint_{self.num}.effort_fb'
        )

    def finish_config(self):
        pass


class HALJointPlumberEC(HALJointPlumber):
    # Quick stop chain:
    #
    # When any drive faults, force low QUICK STOP bit 2 of all drives
    # control word.  The hw_device_mgr is then responsible for stopping
    # enabled drives and waiting for the command to reset the fault.
    #
    # Control word quick stop can be high only if:
    # - no drives in fault AND
    # - hw_device_mgr control word quick stop bit is high
    #
    # Components:
    # - any_fault:  or, 6-inputs, SAFETY_CHAIN + 50
    # - any_fault_s32:  conv_bit_s32, SAFETY_CHAIN + 60
    # - control_word_mask:  muxn_u32, pincount=2, SAFETY_CHAIN + 70
    # - *-control_word_safe:  bitwise, SAFETY_CHAIN + 80
    # - *-cw_bits:  bitslice, SAFETY_CHAIN + 90
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    #                          lcec.0.*.fault -------------> *-fault
    # *-fault -------------+-> any_fault ------------------> any_fault
    # quick_stop ---------/
    # any_fault ----------\
    # 0xffff --------------+-> control_word_mask ----------> control_word_mask
    # 0xfffb -------------/
    #                          hw_device_mgr.*-control_word -> *-control_word
    # control_word_mask --+--> *-control_word_safe --------> *-control_word_safe
    # *-control_word ----<
    #                     \--> *-cw_bits ------------------> *-cw_bit_*
    # *-control_word      ---> lcec.0.*.control_word
    #
    #
    # Feedback chain:
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # (N/A)                lcec.0.*.position-actual-value -> *position-actual-value
    # *position-actual-value -> *_fb_flt                  -> *_drive_fb
    #
    #
    # Command chain:
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # *_ros_pos_cmd       -> *_cmd_s32                  -> *_cmd_s32-out
    # *_cmd_s32-out         -> lcec.0.*.target-position
    #
    #
    # 402 mgr chain:
    #
    # - Pins with names from lcec_to_402_mgr_pins on hw_device_mgr.drive* and
    #   lcec.0.* connected

    def connect_drive(self):
        # fb from drive
        self.newsig('drive_pos_fb', hal.HAL_FLOAT)
        # self.newsig('drive_vel_fb', hal.HAL_FLOAT)
        # self.newsig('drive_acc_fb', hal.HAL_FLOAT)
        # self.newsig('torque_loop_cmd', hal.HAL_FLOAT)
        # self.newsig('torque_cmd', hal.HAL_FLOAT)
        # self.newsig('load', hal.HAL_FLOAT)
        # self.newsig('inertia', hal.HAL_FLOAT)
        # self.newsig('friction', hal.HAL_FLOAT)
        # self.newsig('damping', hal.HAL_FLOAT)
        self.newsig("cw_bit_enable_operation", hal.HAL_BIT)

        # calculate vel and acc cmd from pos cmd
        self.newinst("pll", "cmd_pll")
        self.func_config("cmd_pll.funct", self.Prio.CMD_CHAIN + 1)

        self.newinst("qc", "qc")
        self.func_config("qc.funct", self.Prio.CMD_CHAIN + 2)

        # load parameters from hardware_settings.yaml
        self.pin('cmd_pll.bandwidth').set(self.cmd_bandwidth)
        self.pin('cmd_pll.max-pos').set(self.max_pos)
        self.pin('cmd_pll.min-pos').set(self.min_pos)
        self.pin('cmd_pll.max-vel').set(self.max_vel * 20.0)
        self.pin('cmd_pll.max-acc').set(self.max_acc * 20.0)
        self.pin('cmd_pll.mode').set(2)

        self.signal("cw_bit_enable_operation").link(self.pin("cmd_pll.en"))
        self.signal("jlimits_pos_cmd").link(self.pin("cmd_pll.pos-in"))
        self.signal("ros_vel_cmd").link(self.pin("cmd_pll.vel-in"))
        self.signal("ros_acc_cmd").link(self.pin("cmd_pll.acc-in"))

        self.newsig("pos_c", hal.HAL_FLOAT)
        self.newsig("vel_c", hal.HAL_FLOAT)
        self.newsig("acc_c", hal.HAL_FLOAT)
        self.signal("pos_c").link(self.pin("cmd_pll.pos-out"))
        self.signal("vel_c").link(self.pin("cmd_pll.vel-out"))
        self.signal("acc_c").link(self.pin("cmd_pll.acc-out"))
        self.signal("pos_c").link(self.pin("qc.pos-in"))
        self.signal("vel_c").link(self.pin("qc.vel-in"))
        self.signal("acc_c").link(self.pin("qc.acc-in"))
        self.signal("pos_cmd").link(self.pin("qc.pos-out"))
        self.signal("pos_fb").link(self.pin("qc.pos-fb"))
        self.signal("vel_cmd").link(self.pin("qc.vel-out"))
        self.signal("vel_fb").link(self.pin("qc.vel-fb"))
        self.signal("acc_cmd").link(self.pin("qc.acc-out"))
        self.signal("acc_fb").link(self.pin("qc.acc-fb"))
        self.signal("torque_cmd").link(self.pin("qc.torque-ff"))
        self.signal("torque_fb").link(self.pin("qc.torque-fb"))
        hal.Signal("curr_periodf").link(self.pin("qc.periodf"))
        self.signal("cw_bit_enable_operation").link(self.pin("qc.enable"))
        self.pin('qc.max-pos').set(0.0)
        self.pin('qc.min-pos').set(0.0)
        self.pin('qc.max-vel').set(self.max_vel)
        self.pin('qc.max-acc').set(self.max_acc)

        # calculate vel and acc fb from pos fb
        self.newinst("pll", "fb_pll")
        self.func_config("fb_pll.funct", self.Prio.CMD_CHAIN + 3)

        # load parameters from hardware_settings.yaml
        self.pin('fb_pll.bandwidth').set(self.fb_bandwidth)
        self.pin('fb_pll.max-pos').set(-1.0)
        self.pin('fb_pll.min-pos').set(1.0)
        self.pin('fb_pll.max-vel').set(self.max_vel * 10)
        self.pin('fb_pll.max-acc').set(self.max_acc * 50)
        self.pin('fb_pll.mode').set(3)

        self.signal("cw_bit_enable_operation").link(self.pin("fb_pll.en"))
        self.signal("drive_pos_fb").link(self.pin("fb_pll.pos-in"))
        # self.signal("drive_vel_fb").link(self.pin("fb_pll.vel-in"))
        # self.signal("drive_acc_fb").link(self.pin("fb_pll.acc-in"))
        self.signal("pos_fb").link(self.pin("fb_pll.pos-out"))
        self.signal("vel_fb").link(self.pin("fb_pll.vel-out"))
        self.signal("acc_fb").link(self.pin("fb_pll.acc-out"))
        hal.Signal("curr_periodf").link(self.pin("fb_pll.periodf"))

        # joint feedforward parameter estimation
        # self.newinst("est", "est")
        # self.func_config("est.funct", self.Prio.CMD_CHAIN + 3)

        # self.pin('est.load').set(self.load)
        # self.pin('est.inertia').set(self.inertia)
        # self.pin('est.friction').set(self.friction)
        # self.pin('est.damping').set(self.damping)
        # self.pin('est.load-in').set(self.load)
        # self.pin('est.inertia-in').set(self.inertia)
        # self.pin('est.friction-in').set(self.friction)
        # self.pin('est.damping-in').set(self.damping)
        # self.pin('est.clear').set(self.online_estimation <= 0.0)
        # self.pin('est.max-delta').set(0.25)  # allow +-25% estimation
        # self.pin('est.ji').set(0.0)  # disable inertia estimation
        # self.pin('est.li').set(0.0)  # disable load estimation

        # self.signal("cw_bit_enable_operation").link(self.pin("est.en"))
        # self.signal("load").link(self.pin("est.load"))
        # self.signal("inertia").link(self.pin("est.inertia"))
        # self.signal("friction").link(self.pin("est.friction"))
        # self.signal("damping").link(self.pin("est.damping"))
        # self.signal("vel_cmd").link(self.pin("est.vel-fb"))
        # self.signal("acc_cmd").link(self.pin("est.acc-fb"))
        # self.signal("torque_loop_cmd").link(self.pin("est.torque-fb"))
        # hal.Signal("curr_periodf").link(self.pin("est.periodf"))

        # control loop
        # self.newinst("ppi", "ppi")
        # self.func_config("ppi.funct", self.Prio.CMD_CHAIN + 4)

        # load parameters from hardware_settings.yaml
        # self.pin('ppi.pos-p').set(self.pos_p)
        # self.pin('ppi.vel-p').set(self.vel_p)
        # self.pin('ppi.vel-i').set(self.vel_i)
        # self.pin('ppi.acc-p').set(self.acc_p)
        # self.pin('ppi.acc-i').set(self.acc_i)
        # self.pin('ppi.pos-g').set(self.pos_g)
        # self.pin('ppi.max-pos').set(self.max_pos)
        # self.pin('ppi.min-pos').set(self.min_pos)
        # self.pin('ppi.max-vel').set(self.max_vel * 1.1)
        # self.pin('ppi.max-acc').set(self.max_acc * 1.1)
        # self.pin('ppi.max-torque').set(self.max_torque)
        # self.pin('ppi.torque-boost').set(self.torque_boost)
        # self.pin('ppi.max-pos-error').set(0.1)
        # self.pin('ppi.max-vel-error').set(0.3)
        # self.pin('ppi.max-sat').set(0.5)

        # self.signal("cw_bit_enable_operation").link(self.pin("ppi.en"))
        # self.link_signal('pos_cmd', 'ppi.pos-ff')
        # self.link_signal('vel_cmd', 'ppi.vel-ff')
        # self.link_signal('acc_cmd', 'ppi.acc-ff')
        # self.link_signal('pos_fb', 'ppi.pos-fb')
        # self.link_signal('vel_fb', 'ppi.vel-fb')
        # self.link_signal('acc_fb', 'ppi.acc-fb')
        # self.link_signal('torque_cmd', 'ppi.torque-cmd')
        # self.signal("load").link(self.pin("ppi.load"))
        # self.signal("inertia").link(self.pin("ppi.inertia"))
        # self.signal("friction").link(self.pin("ppi.friction"))
        # self.signal("damping").link(self.pin("ppi.damping"))
        # self.signal("torque_loop_cmd").link(self.pin("ppi.torque-loop-cmd"))
        # hal.Signal("curr_periodf").link(self.pin("ppi.periodf"))

        # connect drive
        self.signal('pos_cmd').link(
            'lcec.0.%i.position_reference' % self.slavenum
        )
        self.signal('vel_cmd').link(
            'lcec.0.%i.velocity_reference' % self.slavenum
        )
        self.signal('torque_cmd').link(
            'lcec.0.%i.torque_reference' % self.slavenum
        )

        # self.signal('drive_pos_fb').link(
        #     'lcec.0.%i.position_actual_value' % self.slavenum
        # )
        # self.signal('drive_vel_fb').link(
        #     'lcec.0.%i.velocity_actual_value' % self.slavenum
        # )
        self.signal('torque_fb').link(
            'lcec.0.%i.torque_actual_value' % self.slavenum
        )

        self.signal('drive_pos_fb').link(
            'lcec.0.%i.position_actual_value' % self.slavenum
        )

        self.signal('pos_err').link(
            'lcec.0.%i.following_error_actual_value' % self.slavenum
        )

        # hal.Pin("lcec.0.%i.velocity_actual_value" % self.slavenum).link(
        #    self.pin("vel_fb_flt.in")
        # self.signal('torque_cmd').link(
        #    'lcec.0.%i.torque_reference' % self.slavenum
        # )

        # Drive control:  Link lcec pins to 402_mgr
        for name in (
            # PDOs
            'control_word',
            'status_word',
            'control_mode',
            'control_mode_fb',
            'error_code',
            *(('sto',) if self.have_sto else tuple()),
        ):
            self.signal(name).link(f"lcec.0.{self.slavenum}.{name}")
        for name in (
            # lcec standard pins
            'online',
            'oper',
        ):
            self.signal(name).link(f"lcec.0.{self.slavenum}.slave-{name}")

        # Link DIOs if no IO module attached
        if not self.params.get("have_itegva", False):
            # Use two digital out from each drive
            do_idx_base = self.slavenum * 2
            for do_idx_drv in (1, 2):
                do_idx = do_idx_base + do_idx_drv
                sig = hal.Signal(f"dout{do_idx:02}")
                sig.link(f"lcec.0.{self.slavenum}.dout{do_idx_drv}")

            # Use digital in 2-5 from first three drives
            if self.slavenum <= 2:
                di_idx_base = self.slavenum * 4
                for di_idx_drv in (2, 3, 4, 5):
                    di_idx = di_idx_base + di_idx_drv - 1
                    sig = hal.Signal(f"din{di_idx:02}")
                    sig.link(f"lcec.0.{self.slavenum}.din{di_idx_drv}")


class HALJointPlumberSim(HALJointPlumber):
    # For sim mode, set up components to simulate the drive moving
    # to the commanded position
    #
    # Simulated motion:
    # - A limit3 comp pretends to move to command position
    #   w/velocity and accel limits
    #
    # Enable chain:
    #
    # - When disabled and zero velocity (sim_drive_load and2 comp),
    #   enable sim motion load pin to accommodate startup position changes
    #
    # - This still allows pretending to be real hardware that takes
    #   time to stop after enable goes low
    #
    # - Sim drive operation_enabled set to True; False doesn't make
    #   sense
    #
    # signal                 -> comp/pin            -> signal
    # ------                    --------               ------
    # (N/A)                sim_startup_selector.load -> sim_drive_load
    #
    # sim_drive_load         -> *_sim_drive.load
    #
    # enable                 -> sim_startup_selector.enable

    def connect_drive(self):
        # Set up simulated start position
        self.newinst("muxnv2", "sim_pos_channel")
        # - Net sim start-up position to sim drive input
        self.newsig("sim_cmd_or_start_pos", hal.HAL_FLOAT)
        self.link_signal("sim_cmd_or_start_pos", "sim_pos_channel.out")
        # - Selector inputs
        hal.Signal("sim_pos_sel").link(self.pin("sim_pos_channel.sel"))
        self.link_signal("jlimits_pos_cmd", "sim_pos_channel.in0")  # cmd pos
        self.set_pin(  # start-up pos
            "sim_pos_channel.in1", self.sim_startup_position
        )
        # - Add update function at drive fb position
        self.func_config("sim_pos_channel.funct", self.Prio.DRIVE_READ_FB + 2)

        # Set up simulated drive motion
        # - Use limit3 comp
        self.newinst("limit3", "sim_drive")
        # - Sim drive joint limits
        self.set_pin("sim_drive.maxv", self.max_vel)
        self.set_pin("sim_drive.maxa", self.max_acc)
        # - Plumb input & output
        self.link_signal("sim_cmd_or_start_pos", "sim_drive.in")
        # self.link_signal("drive_pos_fb", "sim_drive.out")
        # self.link_signal("drive_vel_fb", "sim_drive.out-vel")
        self.link_signal("pos_fb", "sim_drive.out")
        self.link_signal("vel_fb", "sim_drive.out-vel")
        # - Add update function after sim start position
        self.func_config("sim_drive.funct", self.Prio.DRIVE_READ_FB + 10)

        # Set up simulated drive state, mode, fault signals
        sw_sig = self.signal('status_word')
        sw_sig.link(f"hw_device_mgr.0.{self.slavenum}.0.sim_status_word")
        cm_pin = hal.Pin(f"hw_device_mgr.0.{self.slavenum}.0.control_mode_fb")
        cm_pin.unlink()
        del hal.signals[self.jname('control_mode_fb')]
        dmc_sig = self.signal('control_mode')
        dmc_sig.link(cm_pin)
        for name in ('online', 'oper'):
            o_sig = self.signal(name)
            o_sig.link(f"hw_device_mgr.0.{self.slavenum}.0.sim_{name}")
