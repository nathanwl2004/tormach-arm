"""
Generate lines for the ethercat XML file from the known gear ratios and nominal torques.
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
