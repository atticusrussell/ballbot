import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import itertools, numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from meshlib import tris
rng = np.random.default_rng(0)

def sample(T, n):
    a = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
    idx = rng.choice(len(T), n, p=a / a.sum()); u, v = rng.random(n), rng.random(n)
    m = u + v > 1; u[m], v[m] = 1 - u[m], 1 - v[m]
    t = T[idx]; return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])

def pts(shape, n=20000, defl=0.5):
    T, _ = tris(shape.wrapped if hasattr(shape, "wrapped") else shape, defl); return sample(T, n)

def rot24():
    out = []
    for p in itertools.permutations(range(3)):
        for s in itertools.product([1, -1], repeat=3):
            R = np.zeros((3, 3)); R[range(3), p] = s
            if np.isclose(np.linalg.det(R), 1): out.append(R)
    return out

def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0); H = (A - ca).T @ (B - cb); U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T)); D = np.diag([1, 1, d]); R = Vt.T @ D @ U.T
    return R, cb - R @ ca

def icp(src, tree, tgt, R, t, iters=60, trim=0.8):
    for _ in range(iters):
        p = src @ R.T + t; d, i = tree.query(p)
        k = d <= np.quantile(d, trim); dR, dt = kabsch(p[k], tgt[i[k]])
        R, t = dR @ R, dR @ t + dt
    d, _ = tree.query(src @ R.T + t)
    return R, t, d

def register(src, tgt, src_anchor, tgt_anchor, rots=None, tol=0.5):
    tree = cKDTree(tgt); best = None
    for R0 in (rots or rot24()):
        R, t, d = icp(src, tree, tgt, R0, tgt_anchor - R0 @ src_anchor)
        score = (d < tol).mean()
        if best is None or score > best[0]: best = (score, R, t, np.median(d), np.quantile(d, 0.9))
    return best

def H(R, t):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
