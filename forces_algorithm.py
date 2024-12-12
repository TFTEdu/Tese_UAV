import socket
import time
import gps
import math
import threading
import numpy as np

# Constants
EARTH_RADIUS_KM = 6371  # Earth's radius for Haversine distance calculation
IDEAL_DISTANCE_M = 10  # Ideal distance in meters between UAVs
IDEAL_DISTANCE_KM = IDEAL_DISTANCE_M / 1000.0  # Convert to kilometers
BROADCAST_INTERVAL = 1  # Broadcast GPS data every 1 second
CALCULATION_INTERVAL = 5  # Calculate new position every 5 seconds
MAX_MOVEMENT_M = 0.2  # Maximum movement in meters per iteration
MAX_MOVEMENT_KM = MAX_MOVEMENT_M / 1000.0  # Convert to kilometers
DAMPING_FACTOR = 0.2  # Damping factor to reduce oscillations

# Set the batman IP address
batman_ip = '172.27.0.3'

# Function to get GPS data (replace with actual GPS device calls)
def get_gps_data():
    """Retrieves GPS data from the GPSD service."""
    session = gps.gps(mode=gps.WATCH_ENABLE)
    while True:
        try:
            report = session.next()
            if report['class'] == 'TPV':
                latitude = getattr(report, 'lat', None)
                longitude = getattr(report, 'lon', None)
                altitude = getattr(report, 'alt', None)
                speed = getattr(report, 'speed', None)
                if latitude is not None and longitude is not None and latitude != 0 and longitude != 0:
                    return latitude, longitude, altitude, speed
        except Exception as e:
            print(f"Error getting GPS data: {e}")
        time.sleep(0.5)

# Function to calculate Haversine distance between two GPS coordinates
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_KM * c * 1000  # Convert to meters

# Function to calculate force based on distance
def calculate_force(distance, ideal_distance_km):
    if distance > ideal_distance_km:
        return -(distance - ideal_distance_km) * DAMPING_FACTOR  # Attraction
    else:
        return (ideal_distance_km - distance) * DAMPING_FACTOR  # Repulsion

# Function to calculate force vectors between UAVs
def calculate_force_vector(lat1, lon1, lat2, lon2, ideal_distance_km):
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    if distance == 0:
        return 0, 0  # No force if the positions are the same
    force = calculate_force(distance, ideal_distance_km)
    lat_diff = lat2 - lat1
    lon_diff = lon2 - lon1
    norm = np.sqrt(lat_diff ** 2 + lon_diff ** 2)
    lat_diff /= norm
    lon_diff /= norm
    return force * (-lat_diff), force * (-lon_diff)

# Function to limit the movement of a node
def limit_movement(current_lat, current_lon, new_lat, new_lon, max_movement_km):
    distance = haversine_distance(current_lat, current_lon, new_lat, new_lon)
    if distance > max_movement_km:
        scale_factor = max_movement_km / distance
        lat_diff = new_lat - current_lat
        lon_diff = new_lon - current_lon
        return current_lat + lat_diff * scale_factor, current_lon + lon_diff * scale_factor
    return new_lat, new_lon

# Broadcast GPS data to other UAVs
def broadcast_gps_data():
    udp_ip = '172.27.255.255'  # Broadcast address
    udp_port = 5005
    ip_address = batman_ip

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while True:
        try:
            latitude, longitude, altitude, speed = get_gps_data()
            if latitude is not None:
                message = f"{ip_address},{latitude},{longitude},{altitude},{speed}"
                sock.sendto(message.encode(), (udp_ip, udp_port))
                print(f"Broadcasting: {message}")
        except Exception as e:
            print(f"Error while broadcasting: {e}")
        time.sleep(BROADCAST_INTERVAL)

# Listen for incoming GPS data from other UAVs
def listen_for_positions():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 5005))  # Bind to all interfaces on port 5005
    sock.settimeout(2)  # Set timeout for non-blocking listening
    positions = []
    local_ip = batman_ip

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            sender_ip = addr[0]
            if sender_ip == local_ip:
                continue
            ip_address, lat, lon, _, _ = data.decode().split(',')
            lat = float(lat)
            lon = float(lon)
            positions.append((ip_address, lat, lon))
        except socket.timeout:
            break

    return positions

# Main function to run the UAV with the force-based algorithm
def run_uav(ip_address):
    latitude, longitude, altitude, speed = get_gps_data()
    current_position = np.array([latitude, longitude])

    last_broadcast_time = time.time()
    last_calculation_time = time.time()

    broadcast_thread = threading.Thread(target=broadcast_gps_data)
    broadcast_thread.start()

    with open('position_log.txt', 'w') as f:
        while True:
            current_time = time.time()

            if current_time - last_calculation_time >= CALCULATION_INTERVAL:
                neighbors = listen_for_positions()
                if len(neighbors) > 0:
                    total_force_lat, total_force_lon = 0, 0
                    for neighbor in neighbors:
                        force_lat, force_lon = calculate_force_vector(
                            current_position[0], current_position[1],
                            neighbor[1], neighbor[2], IDEAL_DISTANCE_KM)
                        total_force_lat += force_lat
                        total_force_lon += force_lon
                    
                    # Update position
                    new_lat = current_position[0] + total_force_lat
                    new_lon = current_position[1] + total_force_lon
                    new_lat, new_lon = limit_movement(current_position[0], current_position[1], new_lat, new_lon, MAX_MOVEMENT_KM)
                    
                    log_message = f"Current Position: {current_position}\nNeighbors: {neighbors}\nNext Position: [{new_lat}, {new_lon}]\n"
                    print(log_message)
                    f.write(log_message)

                    current_position = np.array([new_lat, new_lon])

                last_calculation_time = current_time

            time.sleep(0.1)

# Run the UAV
if __name__ == "__main__":
    ip_address = batman_ip
    run_uav(ip_address)
