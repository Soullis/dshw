import numpy as np

# DWA Tuning Parameters
HEADING_WEIGHT = 2.0
CLEARANCE_WEIGHT = 1.0
VELOCITY_WEIGHT = 0.2
SIM_TIME = 1.5           # How many seconds into the future to simulate
V_RESOLUTION = 11        # How many velocity steps to check (e.g., 11x11 grid)
COLLISION_RADIUS = 0.5   # Meters. Reject any trajectory that gets closer than this

def calculate_dwa_velocity(
    current_pos: np.ndarray, 
    goal_pos: np.ndarray, 
    bboxes: list, 
    v_max: float
) -> np.ndarray:
    
    # Calculate goal relative to the drone's current position
    goal_rel = goal_pos - current_pos

    # 1. Parse Obstacles from Bounding Boxes into 2D relative coordinates
    obs_points = []
    stride = 5
    n_clusters = len(bboxes) // stride
    
    for i in range(n_clusters):
        idx = i * stride
        depth = bboxes[idx + 4]
        
        if depth <= 0.0:
            continue
            
        # Normalize X center to [-1, 1]
        cx_norm = ((bboxes[idx + 0] + bboxes[idx + 1]) * 0.5 - 160.0) / 160.0
        
        # Approximate lateral Y distance based on a ~60 degree horizontal FOV (tan(30) ≈ 0.57)
        # depth is forward (+X in body frame), y_lat is lateral (+Y is left)
        y_lat = -cx_norm * depth * 0.57 
        obs_points.append(np.array([depth, y_lat]))

    # 2. Create the Dynamic Window (Search Space)
    # Generate an array of possible velocities from -v_max to +v_max
    v_steps = np.linspace(-v_max, v_max, V_RESOLUTION)
    
    best_v = np.zeros(2)
    best_score = -float('inf')

    # 3. Simulate and Evaluate all possible trajectories
    for vx in v_steps:
        for vy in v_steps:
            # Calculate simulated position after SIM_TIME
            sim_pos = np.array([vx * SIM_TIME, vy * SIM_TIME])

            # -- EVALUATE HEADING --
            # Distance from the simulated end-point to the goal
            dist_to_goal = np.linalg.norm(goal_rel[:2] - sim_pos)
            # Invert it so smaller distances yield higher scores
            heading_score = 1.0 / (dist_to_goal + 0.1) 

            # -- EVALUATE CLEARANCE --
            min_dist = float('inf')
            for obs in obs_points:
                dist = np.linalg.norm(obs - sim_pos)
                if dist < min_dist:
                    min_dist = dist
                    
            # Reject trajectory completely if it results in a collision
            if min_dist < COLLISION_RADIUS:
                continue 
                
            clearance_score = min_dist

            # -- EVALUATE VELOCITY --
            # Encourage higher speeds so the drone doesn't stall unnecessarily
            vel_score = np.linalg.norm([vx, vy])

            # -- TOTAL OBJECTIVE FUNCTION --
            score = (HEADING_WEIGHT * heading_score) + \
                    (CLEARANCE_WEIGHT * clearance_score) + \
                    (VELOCITY_WEIGHT * vel_score)

            # Update best velocity if this score is the highest
            if score > best_score:
                best_score = score
                best_v = np.array([vx, vy])

    # Return as 3D vector (vx, vy, vz) - keeping altitude hold (vz=0)
    return np.array([best_v[0], best_v[1], 0.0])