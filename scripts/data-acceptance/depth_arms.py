"""Deterministic depth-source arms shared by dense_gen and the C3 held-out study.

All functions map raw uint16 depth (unit: depth_scale metres) to raw uint16.
"invalid" means 0 or outside [min_depth, max_depth] after scaling.
Arms other than 'none' may change valid pixels (median5, edgegate) or fill
invalid pixels only (nearestfill); the exact behaviour is the object of study.
"""
import cv2
import numpy as np


def valid_mask(raw_u16, min_depth, max_depth, depth_scale):
    lo = int(min_depth / depth_scale)
    hi = int(max_depth / depth_scale)
    return (raw_u16 != 0) & (raw_u16 >= lo) & (raw_u16 <= hi)


def depth_edge_mask(d_m, edge_m=0.1):
    """Depth discontinuity mask; NaN (invalid) never marks an edge."""
    edge = np.zeros(d_m.shape, bool)
    dv = np.abs(np.diff(d_m, axis=0))
    dh = np.abs(np.diff(d_m, axis=1))
    dv = np.nan_to_num(dv, nan=0.0) > edge_m
    dh = np.nan_to_num(dh, nan=0.0) > edge_m
    edge[:-1] |= dv
    edge[1:] |= dv
    edge[:, :-1] |= dh
    edge[:, 1:] |= dh
    return edge


def preprocess(raw_u16, arm, min_depth, max_depth, depth_scale):
    d = raw_u16
    if arm == 'none':
        return d
    invalid = ~valid_mask(d, min_depth, max_depth, depth_scale)
    if arm == 'median5':
        return cv2.medianBlur(d, 5)
    if arm == 'nearestfill':
        if not (invalid.any() and (~invalid).any()):
            return d
        # distanceTransformWithLabels labels every pixel with its nearest ZERO
        # pixel of the input; with labelType=DIST_LABEL_PIXEL (a keyword-only
        # argument: the 4th positional slot is `dst`) each zero pixel owns a
        # unique label. Passing the invalid mask makes valid pixels the zeros,
        # so labels then point at the nearest valid pixel and a per-label value
        # lookup completes the nearest-valid fill. Valid pixels keep their own
        # label at distance 0 and are never modified.
        src_mask = invalid.astype(np.uint8)
        _, labels = cv2.distanceTransformWithLabels(src_mask, cv2.DIST_L2, 5,
                                                    labelType=cv2.DIST_LABEL_PIXEL)
        n_labels = int(labels.max())
        rep = np.zeros(n_labels + 1, d.dtype)
        zeros = src_mask == 0
        rep[labels[zeros]] = d[zeros]
        return rep[labels]
    if arm == 'edgegate':
        d_m = d.astype(np.float32) * depth_scale
        d_m[invalid] = np.nan
        edge = depth_edge_mask(d_m, 0.1)
        gate = cv2.dilate(edge.astype(np.uint8), np.ones((5, 5), np.uint8)) == 0
        med = cv2.medianBlur(d, 5)
        return np.where(gate & (~invalid), med, d)
    raise ValueError(arm)
