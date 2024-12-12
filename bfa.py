import socket
import time
import gps
import math
import threading
import numpy as np

# Constants
EARTH_RADIUS_KM = 6371  # Earth's radius for Haversine distance calculation
MIN_DISTANCE_METERS = 10  # Minimum allowed distance between UAVs
COHESION_FACTOR = 0.05  # Strength of cohesion towards neighbors
SEPARATION_FACTOR = 0.1  # Strength of separation to avoid crowding
BROADCAST_INTERVAL = 1  # Broadcast GPS data every 1 second
CALCULATION_INTERVAL = 5  # Calculate new position every 5 seconds
LOG_FILE = "uav_positions_log_far2.txt"  # Log file to store positions

# Use the actual IP address assigned to the bat0 interface
batman_ip = '172.27.0.4'

# Function to retrieve GPS data from the GPSD service and check if valid
def get_gps_data():
    """Retrieves GPS data from the GPSD service and ensures it's valid."""
    session = gps.gps(mode=gps.WATCH_ENABLE)
    
    while True:  # Keep trying until we get valid GPS data
        try:
            report = session.next()
            if report['class'] == 'TPV':
                latitude = getattr(report, 'lat', None)
                longitude = getattr(report, 'lon', None)
                altitude = getattr(report, 'alt', None)
                speed = getattr(report, 'speed', None)
                
                # Ensure coordinates are not None and not equal to zero
                if latitude is not None and longitude is not None:
                    if latitude != 0 and longitude != 0:
                        return latitude, longitude, altitude, speed
                    else:
                        print("Received invalid coordinates (0, 0). Retrying...")
        except Exception as e:
            print(f"Error getting GPS data: {e}")
        
        time.sleep(0.5)  # Pause briefly before retrying

# Function to get the bat0 IP address
def get_ip_address():
    """Manually sets the IP address of the Raspberry Pi."""
    return "172.27.0.4"  # Replace with the actual IP of bat0

# Function to broadcast GPS data using UDP
def broadcast_gps_data():
    """Broadcasts GPS data with sender's IP address to other Raspberry Pis."""
    udp_ip = '172.27.255.255'  # Broadcast address for the bat0 network
    udp_port = 5005  # Port to send data

    # Manually set the Raspberry Pi's IP address (bat0 IP)
    ip_address = get_ip_address()

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while True:
        try:
            latitude, longitude, altitude, speed = get_gps_data()
            if latitude is not None:
                # Include IP address in the broadcasted message
                message = f"{ip_address},{latitude},{longitude},{altitude},{speed}"
                sock.sendto(message.encode(), (udp_ip, udp_port))
                print(f"Broadcasting: {message}")
        except Exception as e:
            print(f"Unexpected error while broadcasting: {e}")

        time.sleep(BROADCAST_INTERVAL)  # Pause for 1 second between broadcasts


# Function to calculate the distance between two GPS coordinates using the Haversine formula
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS_KM * c * 1000  # Convert to meters
    return distance


# Function to listen for positions broadcast by other nodes
def listen_for_positions():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 5005))  # Bind to all interfaces on port 5005
    sock.settimeout(2)  # Set a short timeout for non-blocking listening
    positions = {}

    # Get the IP address of the current device to filter out self-received messages
    local_ip = get_ip_address()  # Assuming get_ip_address returns the correct bat0 IP
    
    print(f"Listening for incoming positions on {local_ip}...")

    while True:
        try:
            # Wait for data to be received
            data, addr = sock.recvfrom(1024)
            sender_ip = addr[0]
            print(f"Received raw data: {data} from {addr}")

            # Ignore messages from the device's own IP address
            if sender_ip == local_ip:
                print("Ignored self-received message.")
                continue
            
            # Decode and process the data (IP, lat, lon, alt, speed)
            try:
                ip_address, lat, lon, _, _ = data.decode().split(',')
                lat = float(lat)  # Ensure latitude is a float
                lon = float(lon)  # Ensure longitude is a float
                print(f"Processed data - IP: {ip_address}, Latitude: {lat}, Longitude: {lon}")

                # Store the most recent position (only IP, lat, lon)
                positions[ip_address] = (lat, lon)  # We ignore altitude and speed

            except ValueError as e:
                print(f"Error processing received data: {e}")

        except socket.timeout:
            # Timeout case, just continue looping and listening
            print("Socket timed out waiting for data. No positions received.")
            break  # Timeout after no more messages
    
    print(f"Finished listening. Total unique positions received: {len(positions)}")
    return list(positions.values())  # Return only lat, lon as a list of tuples



# Flocking Algorithm: Separation (2D)
def separation(uav_position, neighbors):
    force = np.zeros(2)  # 2D force (lat, lon)
    for neighbor in neighbors:
        # Correct index: neighbor[0] is latitude, neighbor[1] is longitude
        distance = haversine_distance(float(uav_position[0]), float(uav_position[1]), float(neighbor[0]), float(neighbor[1]))
        if distance < MIN_DISTANCE_METERS:
            force += (np.array([uav_position[0], uav_position[1]]) - np.array(neighbor[:2]))  # neighbor[:2] gets lat/lon
            print(f"Separation force applied due to neighbor at distance: {distance} meters")
    return force * SEPARATION_FACTOR


# Flocking Algorithm: Cohesion (2D)
def cohesion(uav_position, neighbors):
    if len(neighbors) > 0:
        latitudes = [neighbor[0] for neighbor in neighbors]  # neighbor[0] is latitude
        longitudes = [neighbor[1] for neighbor in neighbors]  # neighbor[1] is longitude

        # Calculate the center of mass (mean latitude and longitude)
        center_of_mass_lat = np.mean(latitudes)
        center_of_mass_lon = np.mean(longitudes)
        center_of_mass = np.array([center_of_mass_lat, center_of_mass_lon])

        print(f"Calculated center of mass: {center_of_mass}")
        return (center_of_mass - np.array([uav_position[0], uav_position[1]])) * COHESION_FACTOR
    else:
        return np.zeros(2)  # No neighbors, no cohesion force



# Function to log data to a file
def log_positions(file, current_position, neighbors, next_position):
    print("Logging data to file...")  # Debugging print
    with open(file, "a") as f:
        f.write(f"Current Position: {current_position}\n")
        f.write(f"Neighbor Positions:\n")
        for neighbor in neighbors:
            f.write(f"  {neighbor}\n")
        f.write(f"Next Calculated Position: {next_position}\n\n")
    print(f"Logged positions to {file}")  # Debugging print


# Main function for running the Raspberry Pi UAV
def run_uav(ip_address):
    # Wait until we get valid GPS data for the initial position
    print("Waiting for initial GPS data...")
    latitude, longitude, altitude, speed = get_gps_data()
    print(f"Initial GPS data acquired: Lat: {latitude}, Lon: {longitude}")
    current_position = np.array([latitude, longitude])  # Set the initial position from GPS

    last_broadcast_time = time.time()
    last_calculation_time = time.time()

    # Start broadcasting GPS data in a separate thread
    broadcast_thread = threading.Thread(target=broadcast_gps_data)
    broadcast_thread.start()

    while True:  # Continuous loop for logging and broadcasting
        current_time = time.time()

        # Calculate new position based on flocking rules every CALCULATION_INTERVAL seconds
        if current_time - last_calculation_time >= CALCULATION_INTERVAL:
            print("Calculating new position based on flocking rules...")
            neighbors = listen_for_positions()
            if len(neighbors) > 0:
                print(f"Found {len(neighbors)} neighbors. Calculating forces...")
                # Apply separation and cohesion forces to compute the ideal position
                sep_force = separation(current_position, neighbors)
                coh_force = cohesion(current_position, neighbors)
                new_position = current_position + sep_force + coh_force
                print(f"Next position calculated: {new_position}")
                
                # Log positions to file
                log_positions(LOG_FILE, current_position, neighbors, new_position)
                
                                # Update current position
                current_position = new_position

            last_calculation_time = current_time
        
        time.sleep(0.1)  # Prevent busy-waiting

if __name__ == "__main__":
    ip_address = get_ip_address()  # Use the correct IP for the Raspberry Pi
    run_uav(ip_address)

