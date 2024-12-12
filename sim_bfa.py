import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Constants
EARTH_RADIUS_KM = 6371  # Earth's radius for Haversine distance calculation
MIN_DISTANCE_METERS = 10  # Minimum allowed distance between UAVs
COHESION_FACTOR = 0.05  # Strength of cohesion towards neighbors
SEPARATION_FACTOR = 0.1  # Strength of separation to avoid crowding
MAX_MOVE_M = 1  # Max movement per iteration in meters
NUM_ITERATIONS = 10  # Number of iterations to simulate
NUM_NODES = 5  # Number of UAVs

# Function to calculate the Euclidean distance between two points (in meters)
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = EARTH_RADIUS_KM * c * 1000  # Convert to meters
    return distance

# Function to apply separation force
def separation(uav_position, neighbors):
    force = np.zeros(2)
    for neighbor in neighbors:
        distance = haversine_distance(uav_position[0], uav_position[1], neighbor[0], neighbor[1])
        if distance < MIN_DISTANCE_METERS:
            force += (np.array([uav_position[0], uav_position[1]]) - np.array(neighbor[:2]))
    return force * SEPARATION_FACTOR

# Function to apply cohesion force
def cohesion(uav_position, neighbors):
    if len(neighbors) == 0:
        return np.zeros(2)
    center_of_mass = np.mean(neighbors, axis=0)[:2]
    return (center_of_mass - np.array([uav_position[0], uav_position[1]])) * COHESION_FACTOR

# Initialize positions in meters (nodes start within 50 meters from each other)
def initialize_positions_within_50_meters(num_nodes=NUM_NODES):
    return np.random.uniform(low=[40.0, -74.0], high=[40.05, -73.95], size=(num_nodes, 2))

# Simulate UAV movement
def simulate_uav_movement(num_nodes=NUM_NODES, iterations=NUM_ITERATIONS):
    positions = initialize_positions_within_50_meters(num_nodes)
    history = [positions.copy()]

    for iteration in range(iterations):
        new_positions = positions.copy()
        
        # Move the rest of the nodes according to flocking behavior
        for i in range(num_nodes):
            neighbors = [positions[j] for j in range(num_nodes) if j != i]  # All nodes except itself
            sep_force = separation(positions[i], neighbors)
            coh_force = cohesion(positions[i], neighbors)
            new_position = np.array([positions[i][0], positions[i][1]]) + sep_force + coh_force
            new_positions[i][0], new_positions[i][1] = new_position

        positions = new_positions
        history.append(positions.copy())

    return history

# Function to darken a color
def darken_color(color, factor=0.5):
    """Darken the given color by multiplying (1 - factor) with the original RGB values."""
    rgb = mcolors.to_rgb(color)
    return tuple([factor * c for c in rgb])

# Plot the simulation with darker tones for start and end points
def plot_simulation_in_meters(history, num_nodes=NUM_NODES):
    fig, ax = plt.subplots()
    colors = ['red', 'blue', 'green', 'orange', 'purple']

    for node in range(num_nodes):
        x_history = [step[node][0] for step in history]
        y_history = [step[node][1] for step in history]

        # Darken the color for start and end points
        dark_color = darken_color(colors[node])
        
        # Plot the trajectory of the node
        ax.plot(x_history, y_history, color=colors[node], marker='o', label=f'Node {node+1}', alpha=0.6)
        
        # Plot start and end points with a darker tone and larger markers
        ax.plot(x_history[0], y_history[0], color=dark_color, marker='^', markersize=12, label=f'Node {node+1} Start')  # Darker triangle for start
        ax.plot(x_history[-1], y_history[-1], color=dark_color, marker='s', markersize=12, label=f'Node {node+1} End')  # Darker square for end

    ax.set_title("UAV Node Positions Over 10 Iterations")
    ax.set_xlabel("Latitude (degrees)")
    ax.set_ylabel("Longitude (degrees)")
    ax.legend()
    plt.show()

# Run the simulation
history_simulation = simulate_uav_movement()

# Plot the simulation with darker start and end markers
plot_simulation_in_meters(history_simulation)
