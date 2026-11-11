"""Torch-only frame transforms used by the voxel scoring backend."""

import numpy as np
import torch


class KinematicTorch:
    def __init__(self, leg_base_p, device="cpu", dtype=torch.float32):
        base = np.asarray(leg_base_p)
        if base.shape != (3, 6) or not np.isfinite(base).all():
            raise ValueError("leg_base_p must be finite with shape (3,6)")
        if np.any(base[2] != 0):
            raise ValueError("leg frame convention requires zero leg_base_p z")
        self.leg_base_p = torch.tensor(base.T.copy(), device=device, dtype=dtype)
        signs = [[-1, -1, 1]]*3 + [[1, 1, 1]]*3
        self.signs = torch.tensor(signs, device=device, dtype=dtype)
        self.device, self.dtype = self.leg_base_p.device, dtype

    def _R2B(self, points, leg_indices):
        return (points-self.leg_base_p[leg_indices])*self.signs[leg_indices]

    def _B2R(self, points, leg_indices):
        return points*self.signs[leg_indices]+self.leg_base_p[leg_indices]

    def RVectorToLeg(self, vectors, leg_indices):
        return vectors*self.signs[leg_indices]

    LegVectorToR = RVectorToLeg
