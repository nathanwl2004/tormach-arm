# Docker images

This `Dockerfile` builds two images, a developer image with extra
tools meant for developing the packages in this repo, and a
distribution image with the packages in this repo built and installed.

The image builds are divided into several stages.  Stages named with
no `_build` suffix become part of at least one of the two mainline
developer and distribution images.  Those named with the `_build`
suffix are temporary stages that build artifacts to be copied into a
mainline stage.  Some notable characteristics:

- Build inputs and intermediate build objects are thrown away with the
  temporary image
  - Final image size doesn't increase
  - Private sources (robot_ui) don't leak into the final image
- Some of the builds (Qt5, Machinekit) can run in parallel
  - Increases build speed
  - Decreases dirty caches after changes
- Temporary build stages always build for stages with the same parent
  - Helps ensure that everything can be built from a mainline image

The build stage dependency structure looks like this, where
indentation indicates inheritance and "<-" indicates build artifacts
copied from a temporary build stage:

- `base`
  - `ros_base_build`
  - `ros_base` <- `ros_base_build`
    - `ros_devel_build`
    - `ros_custom_build`
    - `ros_custom` <- `ros_custom_build`
      - `ros_d_common_build`
      - `ros_d_common` <- `ros_d_common_build`
        - `ros_dist_build`
        - `ros_dist` <- `ros_dist_build` (published distribution image)
        - `ros_devel` <- `ros_devel_build`
