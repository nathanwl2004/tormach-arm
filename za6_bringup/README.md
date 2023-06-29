# `za6_bringup` package:  Bring up MoveIt and robot HAL hardware

Run ZA HAL hardware and move group:

    ros2 launch za6_bringup bringup.launch

Add optional args:  (for complete list, see `launch/bringup.launch`)

    use_rviz:=false  # Don't launch RViz
    sim_mode:=true  # Run HAL sim hardware
    hal_debug_output:=true  # Log HAL output to console
    hal_debug_level:=5  # Set HAL verbose log level

To command drive state, see the `za6_hardware` package `README.md`.
