"""Learned front end: SuperPoint keypoints/descriptors + LightGlue matching (CPU)."""
import numpy as np
import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import numpy_image_to_torch


class Features:
    """One image's features. Tensors are kept for LightGlue; numpy copies for geometry."""

    def __init__(self, feats, depth, K, depth_scale, max_depth):
        self.feats = feats                                         # dict of tensors (batch 1) for LightGlue
        self.kp = feats["keypoints"][0].numpy().astype(np.float64)  # (N, 2) pixels
        desc = feats["descriptors"][0].numpy()
        g = desc.mean(0)
        self.gdesc = g / (np.linalg.norm(g) + 1e-9)                 # crude global descriptor for place search
        u = np.clip(np.round(self.kp[:, 0]).astype(int), 0, depth.shape[1] - 1)
        v = np.clip(np.round(self.kp[:, 1]).astype(int), 0, depth.shape[0] - 1)
        z = depth[v, u].astype(np.float64) / depth_scale
        self.valid = (z > 0.1) & (z < max_depth)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        self.p3 = np.c_[(self.kp[:, 0] - cx) * z / fx, (self.kp[:, 1] - cy) * z / fy, z]   # camera frame

    def __len__(self):
        return len(self.kp)


class Frontend:
    def __init__(self, K, depth_scale, max_kp=1024, max_depth=6.0, threads=8, resize=480):
        torch.set_num_threads(threads)                 # hybrid CPUs: more threads than P+E sweet spot is slower
        torch.set_grad_enabled(False)
        self.K, self.depth_scale, self.max_depth = K, depth_scale, max_depth
        # SuperPoint on CPU: 640 px wide costs ~900 ms, 480 px ~200 ms (oneDNN kernel behaviour, measured);
        # LightGlue's extract() maps keypoints back to full-resolution pixels, so geometry is unaffected
        self.resize = resize
        self.sp = SuperPoint(max_num_keypoints=max_kp).eval()
        self.lg = LightGlue(features="superpoint").eval()
        self.n_match_calls = 0

    def extract(self, rgb, depth):
        with torch.inference_mode():
            f = self.sp.extract(numpy_image_to_torch(rgb), resize=self.resize)
        return Features(f, depth, self.K, self.depth_scale, self.max_depth)

    def match(self, a, b):
        """Returns (ia, ib): indices of matched keypoints in a and b."""
        self.n_match_calls += 1
        with torch.inference_mode():
            m = self.lg({"image0": a.feats, "image1": b.feats})
        mm = m["matches"][0].numpy()
        return mm[:, 0], mm[:, 1]
