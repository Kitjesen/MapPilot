"""Joint order between MuJoCo and Thunder policies.

MuJoCo groups hip/thigh/calf/wheel per leg; policies group the twelve leg
joints first, followed by the four wheels. Leg order is FR, FL, RR, RL.
"""

import numpy as np

MJ_TO_DART = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
DART_TO_MJ = np.array([0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])
