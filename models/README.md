# UR16e Robotics Toolbox models

`ur16e/DH/UR16e.py` defines the nominal UR16e kinematics used by the G-code
planner. `ur16e/URDF/UR16e.py` loads the matching URDF model for geometry and
forward-kinematics checks.

From the repository root, run `./scripts/install_ur_models.sh` to install both
classes into `.venv` (or the environment selected by `PYTHON_BIN`). The script
generates the UR16e URDF from the installed ROS `ur_description` package, copies
the meshes referenced by that model, registers the classes, and checks that
their forward kinematics agree.

These are model resources installed by the script, not a ROS package. Do not add
this directory to `PYTHONPATH` or replace the library's entire
`models/__init__.py` files.
