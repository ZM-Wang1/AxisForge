import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import glob, os
import sys

if len(sys.argv) < 2:
    print(f"Usage: python {sys.argv[0]} <root_directory>")
    sys.exit(1)

root_dir = sys.argv[1]

if not os.path.isdir(root_dir):
    print(f"Error: Directory not found at path: {root_dir}")
    sys.exit(1)

search_pattern = os.path.join(root_dir, '*_joint.txt')
files = sorted(glob.glob(search_pattern))

if not files:
    raise FileNotFoundError(f"No *_joint.txt files found in the specified directory: {root_dir}")

data = []
for f in files:
    arr = np.loadtxt(f)
    arr = arr[arr <= 20.0]
    filename_only = os.path.basename(f)
    joint = os.path.splitext(filename_only)[0].replace('_joint', '').replace('_', ' ').title()
    data.extend([(joint, x) for x in arr])

df = pd.DataFrame(data, columns=['Joint', 'Latency'])

order = [
    "Shoulder Pan",
    "Shoulder Lift",
    "Elbow",
    "Wrist 1",
    "Wrist 2",
    "Wrist 3"
]

plt.figure(figsize=(12,4))
sns.violinplot(
    data=df,
    x='Joint',
    y='Latency',
    inner='box',
    cut=0,
    order=order
)
plt.ylabel('Latency (ms)')
plt.xlabel('Joint')
plt.title('UR10e Joint Latency Distributions')
plt.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()
plt.show()
