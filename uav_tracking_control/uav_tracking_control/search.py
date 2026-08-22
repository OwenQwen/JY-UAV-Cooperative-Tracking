import math

def generate_lawnmower_waypoints(center_n, center_e, length_n, width_e, spacing, altitude):
    waypoints = []
    half_len = length_n / 2.0
    half_wid = width_e / 2.0
    x_sign = 1
    y = center_e - half_wid
    while y <= center_e + half_wid:
        wp1 = (center_n - half_len * x_sign, y, -altitude)
        wp2 = (center_n + half_len * x_sign, y, -altitude)
        if x_sign == 1:
            waypoints.append(wp1)
            waypoints.append(wp2)
        else:
            waypoints.append(wp2)
            waypoints.append(wp1)
        y += spacing
        x_sign *= -1
    return waypoints

def find_nearest_waypoint_index(pos_n, pos_e, waypoints):
    best_dist = float('inf')
    best_idx = 0
    for i, (wn, we, _) in enumerate(waypoints):
        dist = math.sqrt((wn - pos_n)**2 + (we - pos_e)**2)
        if dist < best_dist:
            best_dist = dist
            best_idx = i
    return best_idx
