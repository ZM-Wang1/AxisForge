#!/bin/bash

SITE_PACKAGES="/home/arkar/.local/lib/python3.10/site-packages"


# --- 1. Copy Robotic Toolbox Model Files (DH Parameters) ---
echo "Copying UR DH model files..."
cp -v local/roboticstoolbox/models/DH/* "$SITE_PACKAGES/roboticstoolbox/models/DH/"

echo "Copying UR URDF model files..."
cp -v local/roboticstoolbox/models/URDF/* "$SITE_PACKAGES/roboticstoolbox/models/URDF/"


# --- 2. Copy RTBData Files (Meshes and URDF) ---
echo "Copying UR10e mesh files..."
cp -rv local/rtbdata/xacro/ur_description/meshes/ur10e "$SITE_PACKAGES/rtbdata/xacro/ur_description/meshes/"

echo "Copying UR URDF description files..."
cp -v local/rtbdata/xacro/ur_description/urdf/* "$SITE_PACKAGES/rtbdata/xacro/ur_description/urdf/"

echo "SUCCESS: All UR files have been copied successfully."