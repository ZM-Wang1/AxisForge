#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import sys

# Read the latency data
data = []
default_file = 'base.txt'
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = default_file
with open(filename, 'r') as f:
    for line in f:
        data.append(float(line.strip()))

data = np.array(data)

# Filter outliers (outside over 20ms)
outlier_count = np.sum((data > 20.0))
filtered_data = data[(data <= 20.0)]

# Create the plot
plt.figure(figsize=(12, 8))

# Time series plot
plt.subplot(2, 1, 1)
plt.plot(filtered_data, 'b-', linewidth=1)
plt.title('Joint Move Latency Over Time')
plt.xlabel('Sample Number')
plt.ylabel('Latency (ms)')
plt.grid(True, alpha=0.3)
plt.axhline(y=np.mean(filtered_data), color='r', linestyle='--', label=f'Mean: {np.mean(filtered_data):.4f} ms')
plt.legend()

# Histogram
plt.subplot(2, 1, 2)
plt.hist(filtered_data, bins=50, alpha=0.7, edgecolor='black')
plt.title('Joint Move Latency Distribution')
plt.xlabel('Latency (ms)')
plt.ylabel('Frequency')
plt.axvline(x=np.mean(filtered_data), color='r', linestyle='--', label=f'Mean: {np.mean(filtered_data):.4f} ms')
plt.axvline(x=np.median(filtered_data), color='g', linestyle='--', label=f'Median: {np.median(filtered_data):.4f} ms')
plt.legend()
plt.grid(True, alpha=0.3)

# Print statistics
print(f"Joint Move Latency Statistics:")
print(f"  Valid Count:  {len(filtered_data)}")
print(f"  Outliers:     {outlier_count}")
print(f"  Mean:         {np.mean(filtered_data):.4f} ms")
print(f"  Median:       {np.median(filtered_data):.4f} ms")
print(f"  Min:          {np.min(filtered_data):.4f} ms")
print(f"  Max:          {np.max(filtered_data):.4f} ms")
print(f"  Std Dev:      {np.std(filtered_data):.4f} ms")
print(f"  IQR:          {np.percentile(filtered_data, 75) - np.percentile(filtered_data, 25):.4f} ms")

plt.tight_layout()
plt.show()