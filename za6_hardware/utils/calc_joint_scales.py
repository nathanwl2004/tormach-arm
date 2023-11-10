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

"""
Generate ethercat XML file lines from known gear ratios and nominal torques.

In the long term it might make sense to use XACRO if we plan to make additional
changes to the XML config, but for now this is enough to at least ensure that
the scale values are calculated and used consistently with less manual copying.
"""
import math

reducer_ratio = [101, 100, 80, 81, 81, -50]
drive_pulley = [24, 1, 1, 28, 30, 1]
driven_pulley = [36, 1, 1, 36, 30, 1]
nominal_torque = [2.39, 2.39, 1.27, 0.32, 0.32, 0.32]

lines = []
for k in range(6):
    gear_ratio = reducer_ratio[k] * driven_pulley[k] / drive_pulley[k]
    fwd_posvel_scale = gear_ratio * (2**23) / (2 * math.pi)
    fwd_torque_scale = 1000.0 / (nominal_torque[k] * gear_ratio)
    inv_posvel_scale = 2.0 * math.pi / (gear_ratio * 2**23)
    inv_torque_scale = nominal_torque[k] * gear_ratio / 1000.0
    lines.append(
        f'          <!-- Convert Joint {k+1} ROS reference values to drive values (forward direction) -->'
    )
    lines.append(
        f'          <!-- gearbox ratio: {gear_ratio:g}, encoder: 23bit, nominal torque: {nominal_torque[k]:g}Nm -->'
    )
    lines.append(
        f'          <!-- forward pos and vel scale = 2**23 * {reducer_ratio[k]:g} * {driven_pulley[k]:g} / {drive_pulley[k]:g} / (2pi) = {fwd_posvel_scale:0.17g} -->'
    )
    lines.append(
        f'          <!-- forward torque scale = 1000.0 / ({gear_ratio:g} * {nominal_torque[k]:g}) = {fwd_torque_scale:0.17g} -->'
    )
    lines.append(
        '          <pdoEntry idx="6040" subIdx="00" bitLen="16" halType="u32" halPin="control-word"/>'
    )
    lines.append(
        f'          <pdoEntry idx="607a" subIdx="00" bitLen="32" scale="{fwd_posvel_scale:0.17g}" offset="0" halType="float" halPin="position-reference"/>'
    )
    lines.append(
        f'          <pdoEntry idx="60b1" subIdx="00" bitLen="32" scale="{fwd_posvel_scale:0.17g}" offset="0" halType="float" halPin="velocity-reference"/>'
    )
    lines.append(
        f'          <pdoEntry idx="60b2" subIdx="00" bitLen="16" scale="{fwd_torque_scale:0.17g}" offset="0" halType="float" halPin="torque-reference"/>'
    )
    lines.append(
        '          <pdoEntry idx="6060" subIdx="00" bitLen="8" halType="u32" halPin="drive-mode-cmd"/>'
    )
    lines.append('')
    lines.append(
        f'          <!-- Convert Joint {k+1} drive feedback values to ROS (inverse direction) -->'
    )
    lines.append(
        f'          <!-- inverse pos and vel scale = 1. / {fwd_posvel_scale:0.17g} = {inv_posvel_scale:0.17g} -->'
    )
    lines.append(
        f'          <!-- inverse torque scale = 1. / {fwd_torque_scale:0.17g} = {inv_torque_scale:0.17g} -->'
    )
    lines.append(
        '          <pdoEntry idx="6060" subIdx="00" bitLen="8" halType="u32" halPin="drive-mode-fb"/>'
    )
    lines.append(
        f'          <pdoEntry idx="6064" subIdx="00" bitLen="32" scale="{inv_posvel_scale:0.17g}" offset="0" halType="float" halPin="position-actual-value"/>'
    )
    lines.append(
        f'          <pdoEntry idx="606c" subIdx="00" bitLen="32" scale="{inv_posvel_scale:0.17g}" offset="0" halType="float" halPin="velocity-actual-value"/>'
    )
    lines.append(
        f'          <pdoEntry idx="6077" subIdx="00" bitLen="16" scale="{inv_torque_scale:0.17g}" offset="0" halType="float" halPin="torque-actual-value"/>'
    )
    lines.append(
        f'          <pdoEntry idx="60f4" subIdx="00" bitLen="32" scale="{inv_posvel_scale:0.17g}" halType="float" halPin="following-error-actual-value"/>'
    )
    lines.append(
        '          <pdoEntry idx="203f" subIdx="00" bitLen="32" halType="u32" halPin="aux-error-code"/>'
    )
    lines.append('')

for line in lines:
    print(line)
