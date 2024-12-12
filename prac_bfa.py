import numpy as np
import matplotlib.pyplot as plt

# Data for positions
current_positions = [
    [38.82349233, -9.295382],
    [38.82349466, -9.29538548],
    [38.82349724, -9.29538354],
    [38.82349981, -9.29538709],
    [38.82350262, -9.29538551],
    [38.82350546, -9.29538917],
    [38.82350879, -9.29538778],
    [38.82351058, -9.29539845],
    [38.82351368, -9.29540321],
]

next_positions = [
    [38.82349466, -9.29538548],
    [38.82349711, -9.29538914],
    [38.82349981, -9.29538709],
    [38.82350305, -9.29537942],
    [38.82350546, -9.29538917],
    [38.82350955, -9.29538181],
    [38.82351172, -9.29539209],
    [38.82351368, -9.29540321],
    [38.82351694, -9.29540821],
]

neighbor_positions = [
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823494472, -9.295501172), (38.823462, -9.2953753)],
    [(38.823485325, -9.295522989), (38.823462, -9.2953753)],
    [(38.82348897, -9.295519303), (38.823462, -9.2953753)],
    [(38.823462, -9.2953753), (38.82348897, -9.295519303)],
]

# Plotting
fig, ax = plt.subplots()

# Plot current positions (blue circles)
for i, current_position in enumerate(current_positions):
    ax.scatter(current_position[1], current_position[0], color='blue', label='Current Position' if i == 0 else "", alpha=0.7)

# Plot next calculated positions (green squares)
for i, next_position in enumerate(next_positions):
    ax.scatter(next_position[1], next_position[0], color='green', marker='s', label='Next Calculated Position' if i == 0 else "", alpha=0.7)

# Plot neighbor positions (red crosses)
for i, neighbors in enumerate(neighbor_positions):
    for neighbor in neighbors:
        ax.scatter(neighbor[1], neighbor[0], color='red', marker='x', label='Neighbor Position' if i == 0 else "", alpha=0.5)

# Connecting current positions to next positions (gray dashed lines)
for i, (current_position, next_position) in enumerate(zip(current_positions, next_positions)):
    ax.plot([current_position[1], next_position[1]], [current_position[0], next_position[0]], color='gray', linestyle='--', alpha=0.8)

# Labels and legend
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_title('UAV Position Movements with Neighbors')

# Remove duplicate labels in legend
handles, labels = ax.get_legend_handles_labels()
unique_labels = dict(zip(labels, handles))

# Improved legend with more descriptive names
ax.legend(unique_labels.values(), unique_labels.keys(), loc='center left', bbox_to_anchor=(1, 0.5), title="Position Types")

plt.tight_layout()
plt.show()
