import numpy as np

from dshw.constants import (
    GOAL_M,
    K_ATT,
    K_REP,
    D0,
    WIDTH,
    HEIGHT
)

def attractive_force(pos: np.ndarray) -> np.ndarray:
    return K_ATT * (GOAL_M - pos)

def repulsive_force(bboxes: list) -> np.ndarray:
    f_total = np.zeros(3)
    if not bboxes:
        return f_total

    stride = 5
    n_clusters = len(bboxes) // stride

    for i in range(n_clusters):
        idx = i * stride
        depth_m = bboxes[idx + 4]
        
        if depth_m <= 0.0 or depth_m >= D0:
            continue

        # APF repulsive magnitude
        inv_d = 1.0 / depth_m
        inv_d0 = 1.0 / D0
        mag = K_REP * (inv_d - inv_d0) * (inv_d ** 2)

        # Normalizing image center offsets to [-1, 1]
        img_cx_norm = ((bboxes[idx + 0] + bboxes[idx + 1]) * 0.5 - (WIDTH / 2)) / (WIDTH / 2)
        img_cy_norm = ((bboxes[idx + 2] + bboxes[idx + 3]) * 0.5 - (HEIGHT / 2)) / (HEIGHT / 2)

        # Repulsion vector in body frame (x=forward, y=left, z=up)
        direction = np.array([-1.0, -img_cx_norm, -img_cy_norm])
        
        # Normalize the direction vector
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm

        f_total += mag * direction

    return f_total