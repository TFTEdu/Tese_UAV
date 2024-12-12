import numpy as np
import matplotlib.pyplot as plt

# Constants
EARTH_RADIUS_KM = 6371
IDEAL_DISTANCE_M = 10  # The ideal distance in meters
IDEAL_DISTANCE_KM = IDEAL_DISTANCE_M / 1000.0  # Convert to kilometers
NUM_ITERATIONS = 100  # Number of iterations to simulate (increased to 25)
MAX_MOVEMENT_M = 0.2  # Maximum allowed movement in meters per iteration
MAX_MOVEMENT_KM = MAX_MOVEMENT_M / 1000.0  # Convert to kilometers
MIN_DISTANCE_M = 8  # Minimum allowed distance in meters between nodes
MIN_DISTANCE_KM = MIN_DISTANCE_M / 1000.0  # Convert to kilometers
MIN_MOVEMENT_THRESHOLD_M = 2  # Minimum movement in meters before stopping node
MIN_MOVEMENT_THRESHOLD_KM = MIN_MOVEMENT_THRESHOLD_M / 1000.0  # Convert to kilometers
DAMPING_FACTOR = 0.2  # Damping factor to reduce oscillations

# Set the range for initial distances between nodes (15 to 25 meters)
MIN_INITIAL_DISTANCE_M = 15
MAX_INITIAL_DISTANCE_M = 20
MIN_INITIAL_DISTANCE_KM = MIN_INITIAL_DISTANCE_M / 1000.0  # Convert to kilometers
MAX_INITIAL_DISTANCE_KM = MAX_INITIAL_DISTANCE_M / 1000.0  # Convert to kilometers

# Haversine formula to calculate the distance between two lat/lon points
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_RADIUS_KM * c

# Force function with damping based on distance to ideal distance
def calculate_force(distance, ideal_distance_km):
    if distance >= ideal_distance_km:
        return -(distance - ideal_distance_km) * DAMPING_FACTOR  # Attraction with damping
    elif distance < MIN_DISTANCE_KM:
        return 100 * DAMPING_FACTOR  # Strong repulsion when too close (with damping)
    else:
        return (ideal_distance_km - distance) * DAMPING_FACTOR  # General repulsion (with damping)

# Function to calculate the force applied by a neighboring node
def calculate_force_vector(lat1, lon1, lat2, lon2, ideal_distance_km):
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    
    if distance == 0:
        return 0, 0  # No force when positions are exactly the same
    
    force = calculate_force(distance, ideal_distance_km)

    # Calculate the difference in position
    lat_diff = lat2 - lat1
    lon_diff = lon2 - lon1
    
    # Normalize the lat/lon difference to create a direction vector
    norm = np.sqrt(lat_diff ** 2 + lon_diff ** 2)

    # If the nodes are too close, make sure we calculate the repulsion correctly
    if norm == 0:
        return 0, 0
    
    # Normalize the directional vector
    lat_diff /= norm
    lon_diff /= norm

    # Apply the force to the normalized direction vector (ensure repulsion direction)
    force_lat = force * (-lat_diff)  # Invert the direction to ensure repulsion
    force_lon = force * (-lon_diff)

    return force_lat, force_lon

# Limit the movement of a node to a maximum distance per iteration, with a minimum threshold for movement
def limit_movement(current_lat, current_lon, new_lat, new_lon, max_movement_km, min_movement_km):
    distance = haversine_distance(current_lat, current_lon, new_lat, new_lon)
    
    # If the movement is smaller than the minimum threshold, do not move the node
    if distance < min_movement_km:
        return current_lat, current_lon
    
    if distance > max_movement_km:
        scale_factor = max_movement_km / distance
        lat_diff = new_lat - current_lat
        lon_diff = new_lon - current_lon
        limited_lat = current_lat + lat_diff * scale_factor
        limited_lon = current_lon + lon_diff * scale_factor
        return limited_lat, limited_lon
    return new_lat, new_lon  # Nunca acontecerá se max_mov maior que min_mov_threshold

# Generate random initial positions with distances between 15 and 25 meters
#def initialize_manual_positions(num_nodes=5):
    base_lat, base_lon = 40.74873, -73.9854  # Start around a base location (e.g., NYC)
    
    positions = np.zeros((num_nodes, 2))
    positions[0] = [base_lat, base_lon]  # Set the first node as the base
    
    for i in range(1, num_nodes):
        # Random distance between 15 and 25 meters (in kilometers)
        distance_km = np.random.uniform(MIN_INITIAL_DISTANCE_KM, MAX_INITIAL_DISTANCE_KM)
        
        # Random angle to spread out the nodes around the base point
        angle = np.random.uniform(0, 2 * np.pi)
        
        # Calculate new lat/lon based on the distance and angle
        delta_lat = distance_km * np.cos(angle) / EARTH_RADIUS_KM * 180 / np.pi
        delta_lon = distance_km * np.sin(angle) / (EARTH_RADIUS_KM * np.cos(np.radians(base_lat))) * 180 / np.pi
        
        new_lat = base_lat + delta_lat
        new_lon = base_lon + delta_lon
        
        positions[i] = [new_lat, new_lon]
    
    return positions

# Initial Positions
def initialize_manual_positions():
    positions = np.array([
        [40.74873, -73.9854],  # Node 1 (very close to Node 2 and Node 3)
        [40.74872, -73.98541],  # Node 2 (within 10 meters of Node 1 and Node 3)
        [40.74873, -73.98543],  # Node 3 (within 10 meters of Node 1 and Node 2)
        [40.74884, -73.98514],  # Node 4 (more than 20 meters away)
        [40.74885, -73.98543]  # Node 5 (more than 20 meters away)
    ])
    return positions

# Simulate the movement of nodes
def simulate_algorithm_manual_positions(num_nodes=5, iterations=NUM_ITERATIONS):
    positions = initialize_manual_positions()  # Initialize nodes with manual positions
    history = [positions.copy()]

    for iteration in range(iterations):
        new_positions = positions.copy()
        
        print(f"Iteration {iteration+1}:")  # Debug: print current iteration

        for i in range(num_nodes):
            total_force_lat, total_force_lon = 0, 0
            for j in range(num_nodes):
                if i != j:
                    # Calculate the force exerted by node j on node i
                    force_lat, force_lon = calculate_force_vector(positions[i][0], positions[i][1], positions[j][0], positions[j][1], IDEAL_DISTANCE_KM)
                    total_force_lat += force_lat
                    total_force_lon += force_lon

                    # Debugging output
                    dist = haversine_distance(positions[i][0], positions[i][1], positions[j][0], positions[j][1])
                    print(f"  Node {i+1} influenced by Node {j+1}: Distance = {dist:.6f} km, Force Lat = {force_lat:.6f}, Force Lon = {force_lon:.6f}")

            # Update the new position based on the total forces
            new_lat = positions[i][0] + total_force_lat
            new_lon = positions[i][1] + total_force_lon
            new_positions[i][0], new_positions[i][1] = limit_movement(positions[i][0], positions[i][1], new_lat, new_lon, MAX_MOVEMENT_KM, MIN_MOVEMENT_THRESHOLD_KM)

        positions = new_positions
        history.append(positions.copy())

    return history

# Function to darken the original color
def darken_color(color, factor=0.7):
    """Darken the given color by multiplying (1 - factor) with the original RGB values."""
    rgb = mcolors.to_rgb(color)
    return tuple([factor * c for c in rgb])

# Plot the simulation of positions with darker tones for start and end points
def plot_simulation(history, num_nodes=5):
    fig, ax = plt.subplots()
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Plot the movement of the regular nodes
    for node in range(num_nodes):
        lat_history = [step[node][0] for step in history]
        lon_history = [step[node][1] for step in history]
        
        # Darken the color for start and end points
        dark_color = darken_color(colors[node])
        
        # Plot the trajectory of the node
        ax.plot(lon_history, lat_history, color=colors[node], marker='o', label=f'Node {node+1}', alpha=0.6)
        
        # Plot start and end points with a darker tone and larger markers
        ax.plot(lon_history[0], lat_history[0], color=dark_color, marker='^', markersize=12, label=f'Node {node+1} Start')  # Darker triangle for start
        ax.plot(lon_history[-1], lat_history[-1], color=dark_color, marker='s', markersize=12, label=f'Node {node+1} End')  # Darker square for end
    
    ax.set_title("Simulated Raspberry Pi Node Positions Over Iterations")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.legend()
    plt.show()

# Run the simulation with manual starting positions
history_manual_positions = simulate_algorithm_manual_positions()

# Plot the simulation with darker start and end markers
plot_simulation(history_manual_positions, num_nodes=5)
