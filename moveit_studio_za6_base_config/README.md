# `moveit_studio_za6_base_config`

This package contains the base config to operate a Tormach ZA6 using
MoveIt Studio.


## Installing MoveIt Studio

Install MoveIt Studio using the ["Quick Start" documentation][1].
Test it with the standard UR5 configuration to be sure it works.

The install will create a workspace directory, `moveit_studio_ws` by
default.  Check out this repo into the workspace directory's `src/`
subdirectory.

Edit the `.env` file in the top directory and set the Studio default
config package:

    STUDIO_CONFIG_PACKAGE=moveit_studio_za6_base_config

For the PCNC1100 machine tending configuration:

    STUDIO_CONFIG_PACKAGE=moveit_studio_za6_tending_config

Edit the `docker-compose.yaml` file and pass the `ZA_GRIPPER`
environment variable to containers:

    services:
      base:
        [...]
        environment:
          [...]
          # Pass in the ZA_GRIPPER option
          - ZA_GRIPPER=${ZA_GRIPPER:-none}

Now go to the next section to start Studio with mock hardware, or the
section after that to start Studio with sim or real hardware.

[1]: https://docs.picknik.ai/en/stable/getting_started/setup_tutorials/install_software/software_installation.html


## Launch MoveIt Studio with mock hardware

MoveIt Studio can run with mock hardware, where a separate ZA6 Docker
container isn't needed.  This can be useful for developing in Studio
or testing URDF and other changes in Studio or without having to
restart two containers.

    # Build the Studio workspace
    ./moveit_studio build -v

    # Launch Studio with mock hardware
    MOCK_HARDWARE=true ./moveit_studio run -v

The `MOCK_HARDWARE=true` variable can be set more permanently from the
`.env` file.

Optionally, add a gripper by prepending the last command with
e.g. `ZA_GRIPPER=pivot_gripper`.  Valid gripper names come from the
`za6_tools` package.

    MOCK_HARDWARE=true ZA_GRIPPER=pivot_gripper ./moveit_studio run -v


## Launch MoveIt Studio with ZA6

To launch MoveIt Studio with a ZA6, either sim or real hardware, a
separate ZA6 container must run next to the Studio containers.

- In a terminal, start the ZA6 container and start the robot as
  described in this repo's top-level `README.md`:

        # Change to this repo's top-level directory
        cd moveit_studio_ws/src/tormach_za6_studio_config

        # Start the ZA Docker container; add 'sim' if needed
        ./launch_za_dist_image.sh # sim

        # Start the robot; add 'sim_mode:=true' if needed
        ros2 launch za6_bringup bringup.launch # sim_mode:=true

- In a new terminal, build and run the MoveIt Studio workspace:

        # Optionally remove old build, if stale files cause problems
        rm -rf ~/.config/moveit_studio

        # Build the ZA6 studio config from the moveit_studio directory
        ./moveit_studio build

        # Start MoveIt Studio without launching drivers
        # (launched in the ZA6 container)
        ./moveit_studio run -v --no-drivers
