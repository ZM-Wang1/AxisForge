# Robotics Toolbox model overlays

`roboticstoolbox/` contains the custom DH and URDF model overlays. `rtbdata/`
contains the accompanying UR10e xacro and mesh assets. These are model resources,
not a replacement installation of Robotics Toolbox; do not add this directory
to `PYTHONPATH` or replace the library's entire `models/__init__.py` files.

From the repository root, run `./scripts/install_ur_models.sh` to install the
UR16e classes into `.venv` (or the environment selected by `PYTHON_BIN`). The
script generates the UR16e URDF from the installed ROS `ur_description` package,
copies its referenced meshes, registers the classes, and checks DH/URDF forward
kinematics. The UR10e resources are retained separately.

The `ur_control` package also installs these resources under
`share/ur_control/models/`.
