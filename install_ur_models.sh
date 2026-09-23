#!/bin/bash
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-$PROJECT_DIR/.venv/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Project Python not found: $PYTHON_BIN. Set PYTHON_BIN to the intended environment." >&2
    exit 1
fi
source /opt/ros/humble/setup.bash

# Project environment: roboticstoolbox-python==1.1.1, spatialmath-python==1.1.9,
# spatialgeometry==1.1.0, numpy==1.26.4, scipy==1.11.4, matplotlib==3.5.1.
"$PYTHON_BIN" - "$PROJECT_DIR" <<'PY'
from pathlib import Path
import math
import shutil
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import roboticstoolbox
import rtbdata
import xacro

project = Path(sys.argv[1])
models = Path(roboticstoolbox.__file__).parent / 'models'
data = Path(rtbdata.__file__).parent / 'xacro' / 'axisforge_ur16e'
description = Path(get_package_share_directory('ur_description'))

# Expand the official UR16e geometry and nominal kinematics, not UR10e data.
document = xacro.process_file(
    str(description / 'urdf' / 'ur.urdf.xacro'),
    mappings={'name': 'ur16e', 'ur_type': 'ur16e'},
)
root = ET.fromstring(document.toxml())

# Express the model in the controller's Base frame, matching the DH model.
# ROS base_link differs from that frame by a pi rotation about Z.
root.find("joint[@name='base_joint']/origin").set('rpy', f'0 0 {math.pi}')

# Keep tool0 as the sole leaf so Toolbox fkine(q) selects the proper endpoint.
for element in list(root):
    if element.tag not in ('link', 'joint', 'material'):
        root.remove(element)
    elif element.tag == 'link' and element.get('name') in ('base', 'ft_frame'):
        root.remove(element)
    elif element.tag == 'joint' and element.find('child').get('link') in ('base', 'ft_frame'):
        root.remove(element)

# Copy only meshes referenced by the official UR16e model into its own folder.
# UR16e shares base/wrist meshes with UR10e; its arm meshes are UR16e-specific.
for mesh in root.iter('mesh'):
    filename = mesh.get('filename')
    prefix = 'package://ur_description/'
    if not filename.startswith(prefix):
        raise RuntimeError(f'Unexpected mesh URI: {filename}')
    relative = Path(filename[len(prefix):])
    destination = data / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(description / relative, destination)
    mesh.set('filename', str(destination))

data.mkdir(parents=True, exist_ok=True)
ET.indent(root)
ET.ElementTree(root).write(data / 'ur16e.urdf', encoding='utf-8', xml_declaration=True)

for kind in ('DH', 'URDF'):
    target = models / kind
    shutil.copy2(project / 'local' / 'roboticstoolbox' / 'models' / kind / 'UR16e.py', target / 'UR16e.py')
    init = target / '__init__.py'
    text = init.read_text()
    import_line = f'from roboticstoolbox.models.{kind}.UR16e import UR16e'
    if import_line not in text:
        backup = init.with_name('__init__.py.before_ur16e')
        if not backup.exists():
            shutil.copy2(init, backup)
        init.write_text(text + '\n' + import_line + '\n__all__.append("UR16e")\n')

print(f'Installed UR16e DH and URDF models into {models}')
print(f'Installed UR16e URDF and referenced meshes into {data}')
PY

# A fresh interpreter is needed to pick up the newly registered classes.
"$PYTHON_BIN" - <<'PY'
import numpy as np
import roboticstoolbox as rtb

dh = rtb.models.DH.UR16e()
urdf = rtb.models.UR16e()
assert dh.n == urdf.n == 6
assert urdf.ee_links[0].name == 'tool0'
poses = [np.zeros(6), dh.qr, np.deg2rad([-3.73, -90, -120, -60, 90, 0])]
for q in poses:
    np.testing.assert_allclose(dh.fkine(q).A, urdf.fkine(q).A, atol=1e-8)
print('SUCCESS: Both UR16e models load and their forward kinematics agree.')
PY
