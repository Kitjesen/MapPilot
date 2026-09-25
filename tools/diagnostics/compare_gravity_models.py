"""Offline model comparison using existing SciPy; never writes a map.

Keeps LingTu measurements/information fixed while comparing unrestricted SE(3)
with soft gravity against yaw + XYZ variables with fixed observed gravity.
This is an independent numerical reference, not a GTSAM or VINS execution.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def compare(poses_path, constraints_path):
    rows = [line.split() for line in poses_path.read_text().splitlines() if line.strip()]
    poses = np.array([[float(v) for v in row[1:]] for row in rows])
    positions = poses[:, :3]
    reference = Rotation.from_quat(poses[:, [4, 5, 6, 3]])
    edges = np.loadtxt(constraints_path, skiprows=4, ndmin=2)
    source, target = edges[:, 0].astype(int), edges[:, 1].astype(int)
    measurements = Rotation.from_quat(edges[:, [6, 7, 8, 5]])
    translation = edges[:, 2:5]
    information = np.zeros((len(edges), 6, 6))
    upper = np.triu_indices(6)
    information[:, upper[0], upper[1]] = edges[:, 9:]
    information[:, upper[1], upper[0]] = edges[:, 9:]
    values, vectors = np.linalg.eigh(information)
    whitening = np.sqrt(np.maximum(values, 0))[:, :, None] * vectors.transpose(0, 2, 1)
    observed_gravity = reference.inv().apply(np.tile([0., 0., 1.], (len(poses), 1)))

    def decode(x, mode):
        if mode == 'se3_soft_gravity':
            state = np.vstack([np.r_[reference[0].as_rotvec(), positions[0]], x.reshape(-1, 6)])
            return Rotation.from_rotvec(state[:, :3]), state[:, 3:]
        state = np.vstack([np.r_[0., positions[0]], x.reshape(-1, 4)])
        return Rotation.from_rotvec(np.column_stack([state[:, 0]*0, state[:, 0]*0, state[:, 0]])) * reference, state[:, 1:]

    def residual(x, mode):
        rotation, position = decode(x, mode)
        error_rotation = measurements.inv() * rotation[source].inv() * rotation[target]
        omega = error_rotation.as_rotvec()
        error_translation = measurements.inv().apply(
            rotation[source].inv().apply(position[target] - position[source]) - translation)
        theta = np.linalg.norm(omega, axis=1)
        coefficient = np.full_like(theta, 1./12.)
        mask = theta > 1e-4
        coefficient[mask] = (1. - .5*theta[mask]/np.tan(.5*theta[mask])) / theta[mask]**2
        cross = np.cross(omega, error_translation)
        upsilon = error_translation - .5*cross + coefficient[:, None]*np.cross(omega, cross)
        edge_error = np.einsum('nij,nj->ni', whitening, np.column_stack([omega, upsilon])).ravel()
        if mode == 'se3_soft_gravity':
            attitude = (reference.inv()*rotation).as_rotvec()
            perpendicular = attitude - np.sum(attitude*observed_gravity, axis=1)[:, None]*observed_gravity
            return np.r_[edge_error, (perpendicular/.01).ravel()]
        return edge_error

    results = {}
    for mode in ['se3_soft_gravity', 'fixed_gravity_4dof']:
        if mode == 'se3_soft_gravity':
            initial = np.column_stack([reference.as_rotvec(), positions])[1:].ravel()
        else:
            initial = np.column_stack([np.zeros(len(poses)), positions])[1:].ravel()
        initial_error = residual(initial, mode)
        solved = least_squares(residual, initial, args=(mode,), method='lm',
                               ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=1000)
        rotation, position = decode(solved.x, mode)
        gravity = rotation.inv().apply(np.tile([0., 0., 1.], (len(poses), 1)))
        angles = np.arctan2(np.linalg.norm(np.cross(gravity, observed_gravity), axis=1),
                            np.sum(gravity*observed_gravity, axis=1))
        results[mode] = dict(success=bool(solved.success), nfev=solved.nfev,
                             initial_cost=float(.5*initial_error@initial_error), cost=float(solved.cost),
                             max_gravity_error_rad=float(angles.max()),
                             height_span_m=float(np.ptp(position[:, 2])),
                             endpoint_height_delta_m=float(position[-1, 2]-position[0, 2]))
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses', type=Path)
    parser.add_argument('constraints', type=Path)
    parser.add_argument('report', type=Path)
    args = parser.parse_args()
    report = compare(args.poses, args.constraints)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
