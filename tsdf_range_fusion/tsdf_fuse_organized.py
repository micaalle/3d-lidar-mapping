from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

try:
    from skimage import measure
except Exception as e:
    raise RuntimeError("Missing scikit-image. Install with: pip install scikit-image") from e


DEFAULT_CSV = Path("scan_output/room_scan.csv")
DEFAULT_OUTDIR = Path("tsdf_output")


@dataclass
class TsdfReport:
    source_csv: str
    raw_rows: int
    valid_rows: int
    binned_rays: int
    integrated_rays: int
    voxel_size_m: float
    truncation_m: float
    grid_min: list[float]
    grid_max: list[float]
    grid_shape: list[int]
    mesh_vertices: int
    mesh_triangles: int
    notes: list[str]


# csv load ./ filter

def load_scan_csv(csv_path: Path) -> dict[str, np.ndarray]:
    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                rows.append(row)

    if not rows:
        raise RuntimeError(f"CSV is empty: {csv_path}")

    first = [c.strip().lower() for c in rows[0]]
    has_header = all(k in first for k in ["x", "y", "z"])

    parsed = {
        "segment": [],
        "timestamp": [],
        "tilt": [],
        "azimuth": [],
        "distance": [],
        "quality": [],
        "x": [],
        "y": [],
        "z": [],
    }

    if has_header:
        idx = {name: first.index(name) for name in first}
        data_rows = rows[1:]

        def get(row, name, default):
            try:
                return float(row[idx[name]])
            except Exception:
                return default

        for i, row in enumerate(data_rows):
            try:
                x = float(row[idx["x"]])
                y = float(row[idx["y"]])
                z = float(row[idx["z"]])
            except Exception:
                continue
            # format from scan:
            # segment_id,host_timestamp_ns,tilt_deg,azimuth_deg,distance_m,quality,start_flag,x,y,z
            parsed["segment"].append(int(get(row, "segment_id", 0)))
            parsed["timestamp"].append(get(row, "host_timestamp_ns", i))
            parsed["tilt"].append(get(row, "tilt_deg", np.nan))
            parsed["azimuth"].append(get(row, "azimuth_deg", np.nan))
            parsed["distance"].append(get(row, "distance_m", np.linalg.norm([x, y, z])))
            parsed["quality"].append(get(row, "quality", 0))
            parsed["x"].append(x)
            parsed["y"].append(y)
            parsed["z"].append(z)
    else:   
        for i, row in enumerate(rows):
            if len(row) < 10:
                continue
            try:
                parsed["segment"].append(int(float(row[0])))
                parsed["timestamp"].append(float(row[1]))
                parsed["tilt"].append(float(row[2]))
                parsed["azimuth"].append(float(row[3]))
                parsed["distance"].append(float(row[4]))
                parsed["quality"].append(float(row[5]))
                parsed["x"].append(float(row[-3]))
                parsed["y"].append(float(row[-2]))
                parsed["z"].append(float(row[-1]))
            except Exception:
                continue

    out = {k: np.asarray(v) for k, v in parsed.items()}
    out["points"] = np.column_stack([out["x"], out["y"], out["z"]]).astype(np.float64)
    out["raw_rows"] = np.asarray([len(rows)], dtype=np.int64)
    return out

# a little cleanin for outliers works but i havent really had any noisy scans to test it 
def filter_scan(data: dict[str, np.ndarray], args) -> dict[str, np.ndarray]:
    points = data["points"]
    finite = np.all(np.isfinite(points), axis=1)

    dist_from_xyz = np.linalg.norm(points - np.asarray(args.sensor_origin, dtype=np.float64), axis=1)

    keep = finite
    keep &= dist_from_xyz >= args.min_range
    keep &= dist_from_xyz <= args.max_range

    if "distance" in data:
        d = data["distance"]
        keep &= np.isfinite(d)
        keep &= d >= args.min_range
        keep &= d <= args.max_range

    if args.clean_percentile < 100:
        center = np.median(points[keep], axis=0)
        r = np.linalg.norm(points - center, axis=1)
        keep &= r <= np.percentile(r[keep], args.clean_percentile)

    out = {}
    for k, v in data.items():
        if isinstance(v, np.ndarray) and len(v) == len(points):
            out[k] = v[keep]
        else:
            out[k] = v
    out["points"] = out["points"].astype(np.float64)
    return out


# organized rays

def build_organized_rays(data: dict[str, np.ndarray], args) -> tuple[np.ndarray, np.ndarray, dict]:
    """
    raw into organized angular cells.

    segment_id, round(tilt / tilt_bin_deg), round(azimuth / azimuth_bin_deg)

    Within each cell, choose a representative ray:
      median range by default
      nearest range if --bin-stat closest
      farthest range if --bin-stat farthest

    Returns:
      ray_points: representative measured 3D points
      ray_dirs: normalized direction vectors from sensor origin
      metadata
    """
    points = data["points"]
    segment = data["segment"].astype(np.int64) if "segment" in data else np.zeros(len(points), dtype=np.int64)
    tilt = data["tilt"]
    az = data["azimuth"]

    origin = np.asarray(args.sensor_origin, dtype=np.float64)
    ranges = np.linalg.norm(points - origin[None, :], axis=1)
    # if missing just random sample 
    has_angles = np.isfinite(tilt) & np.isfinite(az)

    if not np.any(has_angles):
        print("Warning: no valid tilt/azimuth columns. Falling back to voxel/range sampling only.")
        return sample_rays_without_angles(points, origin, args.max_rays)

    points = points[has_angles]
    segment = segment[has_angles]
    tilt = tilt[has_angles]
    az = az[has_angles] % 360.0
    ranges = ranges[has_angles]

    tilt_idx = np.round(tilt / args.tilt_bin_deg).astype(np.int64)
    az_idx = np.round(az / args.azimuth_bin_deg).astype(np.int64)

    if args.fuse_segments_before_tsdf:
        # merge
        segment_key = np.zeros_like(segment)
    else:
        segment_key = segment

    order = np.lexsort((az_idx, tilt_idx, segment_key))
    segment_key = segment_key[order]
    tilt_idx = tilt_idx[order]
    az_idx = az_idx[order]
    ranges = ranges[order]
    points = points[order]


    keys = np.column_stack([segment_key, tilt_idx, az_idx])
    change = np.ones(len(keys), dtype=bool)
    change[1:] = np.any(keys[1:] != keys[:-1], axis=1)
    starts = np.where(change)[0]
    ends = np.r_[starts[1:], len(keys)]

    chosen = []
    # segments of same keys 
    for s, e in zip(starts, ends):
        group_ranges = ranges[s:e]
        if len(group_ranges) == 0:
            continue

        if args.bin_stat == "closest":
            local = int(np.argmin(group_ranges))
        elif args.bin_stat == "farthest":
            local = int(np.argmax(group_ranges))
        else:
            med = np.median(group_ranges)
            local = int(np.argmin(np.abs(group_ranges - med)))

        chosen.append(s + local)

    chosen = np.asarray(chosen, dtype=np.int64)
    ray_points = points[chosen]

    if args.max_rays > 0 and len(ray_points) > args.max_rays:
        rng = np.random.default_rng(args.random_seed)
        idx = rng.choice(len(ray_points), size=args.max_rays, replace=False)
        ray_points = ray_points[np.sort(idx)]

    vectors = ray_points - origin[None, :]
    ray_ranges = np.linalg.norm(vectors, axis=1)
    valid = ray_ranges > 1e-6
    ray_points = ray_points[valid]
    ray_ranges = ray_ranges[valid]
    ray_dirs = vectors[valid] / ray_ranges[valid, None]

    meta = {
        "input_valid_angle_samples": int(np.sum(has_angles)),
        "organized_bins": int(len(chosen)),
        "integrated_after_limit": int(len(ray_points)),
        "tilt_bin_deg": float(args.tilt_bin_deg),
        "azimuth_bin_deg": float(args.azimuth_bin_deg),
        "bin_stat": args.bin_stat,
        "fuse_segments_before_tsdf": bool(args.fuse_segments_before_tsdf),
    }

    return ray_points, ray_dirs, meta

# range instead of angles if any missin
def sample_rays_without_angles(points: np.ndarray, origin: np.ndarray, max_rays: int) -> tuple[np.ndarray, np.ndarray, dict]:
    points = points.copy()
    if max_rays > 0 and len(points) > max_rays:
        rng = np.random.default_rng(7)
        idx = rng.choice(len(points), size=max_rays, replace=False)
        points = points[np.sort(idx)]

    vectors = points - origin[None, :]
    ranges = np.linalg.norm(vectors, axis=1)
    valid = ranges > 1e-6

    return points[valid], vectors[valid] / ranges[valid, None], {
        "input_valid_angle_samples": 0,
        "organized_bins": int(len(points)),
        "integrated_after_limit": int(np.sum(valid)),
        "fallback": "no_angles",
    }


# TSDF integration
# Real-Time Dense 3D Scene Reconstruction: Fusing LiDAR Data into the TSDF, Author Moritz Miebach
# https://www.ais.uni-bonn.de/theses/Moritz_Miebach_Bachelor_Thesis_07_2021.pdf
# this was a great help for inital TSDF understanding but I found it gave too large of holes for my scans so I just do basic ray sampling 
# it honestly looks super messy and some strogner smoothing should be applied but for the purpose of giving a bit of clarity in understanding the room contruction if the points are enough, is enough of a suppliment rather then a perfect recreation of the room
# theres also the issue of shadows which make it super messy, so i think the final best version of this would use multiple scans with different csvs all combined together to make the mesh.
# with my current lidar setup I dont have any way of checking movement form one spot to another but seeing as how it could make my lidar scans better I like the idea of doing that for a version 2 of the physical lidar setup and revisiting TSDF fusion with multiple scans from different positions.
# With that implemnted following the papers ideas would be a lot more helpful as it does addresss that idea. 


def compute_bounds(points: np.ndarray, args) -> tuple[np.ndarray, np.ndarray]:
    if args.bounds is not None:
        vals = np.asarray(args.bounds, dtype=np.float64)
        if vals.shape != (6,):
            raise ValueError("--bounds expects 6 numbers: xmin xmax ymin ymax zmin zmax")
        mn = np.array([vals[0], vals[2], vals[4]], dtype=np.float64)
        mx = np.array([vals[1], vals[3], vals[5]], dtype=np.float64)
    else:
        lo = np.percentile(points, args.bounds_percentile_low, axis=0)
        hi = np.percentile(points, args.bounds_percentile_high, axis=0)
        mn = lo - args.bounds_padding
        mx = hi + args.bounds_padding

    return mn, mx


def make_grid(points: np.ndarray, args):
    mn, mx = compute_bounds(points, args)

    origin = np.asarray(args.sensor_origin, dtype=np.float64)
    mn = np.minimum(mn, origin - args.bounds_padding)
    mx = np.maximum(mx, origin + args.bounds_padding)

    size = mx - mn
    shape = np.ceil(size / args.voxel_size).astype(np.int64) + 1

    total = int(np.prod(shape))
    if total > args.max_voxels:
        raise RuntimeError(
            f"Requested TSDF grid is too large: shape={shape.tolist()} voxels={total:,}. "
            f"Increase --voxel-size, set --bounds, or increase --max-voxels."
        )

    tsdf = np.ones(total, dtype=np.float32)
    weights = np.zeros(total, dtype=np.float32)

    return mn.astype(np.float64), shape.astype(np.int64), tsdf, weights


def points_to_flat_indices(points: np.ndarray, grid_min: np.ndarray, shape: np.ndarray, voxel: float):
    ijk = np.floor((points - grid_min[None, :]) / voxel).astype(np.int64)
    valid = np.all(ijk >= 0, axis=1) & np.all(ijk < shape[None, :], axis=1)
    ijk = ijk[valid]

    # flat index for volume indexed [x, y, z]
    flat = ijk[:, 0] * (shape[1] * shape[2]) + ijk[:, 1] * shape[2] + ijk[:, 2]
    return flat.astype(np.int64), valid


def integrate_one_ray(
    point: np.ndarray,
    ray_dir: np.ndarray,
    origin: np.ndarray,
    grid_min: np.ndarray,
    shape: np.ndarray,
    voxel_size: float,
    truncation: float,
    tsdf: np.ndarray,
    weights: np.ndarray,
    args,
):
    r = float(np.linalg.norm(point - origin))
    if r <= args.min_range:
        return 0

    surface_start = max(args.min_range, r - truncation)
    surface_end = r + truncation

    if args.integrate_free_space:
        # free-space samples are lower weight, they keep empty space from turning into junk surfaces.
        free_end = max(args.min_range, r - truncation)
        if free_end > args.min_range:
            t_free = np.arange(args.min_range, free_end, args.free_space_step)
            if len(t_free):
                pts_free = origin[None, :] + t_free[:, None] * ray_dir[None, :]
                flat, valid = points_to_flat_indices(pts_free, grid_min, shape, voxel_size)
                if len(flat):
                    vals = np.ones(len(flat), dtype=np.float32)
                    w = np.full(len(flat), args.free_space_weight, dtype=np.float32)
                    update_tsdf(tsdf, weights, flat, vals, w, args.max_weight)

    t = np.arange(surface_start, surface_end + 1e-9, args.surface_step)
    if len(t) == 0:
        return 0

    pts = origin[None, :] + t[:, None] * ray_dir[None, :]
    flat, valid = points_to_flat_indices(pts, grid_min, shape, voxel_size)
    if len(flat) == 0:
        return 0

    t_valid = t[valid]
    sdf = r - t_valid
    sdf_norm = np.clip(sdf / truncation, -1.0, 1.0).astype(np.float32)

    obs_w = np.ones(len(sdf_norm), dtype=np.float32) * args.surface_weight

    # remove dupes within same ray
    if len(flat) > 1:
        order = np.argsort(flat)
        flat_s = flat[order]
        sdf_s = sdf_norm[order]
        w_s = obs_w[order]

        unique_starts = np.r_[0, np.where(flat_s[1:] != flat_s[:-1])[0] + 1]
        unique_ends = np.r_[unique_starts[1:], len(flat_s)]
        keep_indices = []
        for a, b in zip(unique_starts, unique_ends):
            local = a + int(np.argmin(np.abs(sdf_s[a:b])))
            keep_indices.append(local)

        keep_indices = np.asarray(keep_indices, dtype=np.int64)
        flat = flat_s[keep_indices]
        sdf_norm = sdf_s[keep_indices]
        obs_w = w_s[keep_indices]

    update_tsdf(tsdf, weights, flat, sdf_norm, obs_w, args.max_weight)
    return len(flat)


def update_tsdf(tsdf: np.ndarray, weights: np.ndarray, flat: np.ndarray, values: np.ndarray, obs_w: np.ndarray, max_weight: float):
    if len(flat) == 0:
        return

    old_w = weights[flat]
    old_v = tsdf[flat]

    new_w = np.minimum(old_w + obs_w, max_weight)
    tsdf[flat] = (old_v * old_w + values * obs_w) / np.maximum(new_w, 1e-9)
    weights[flat] = new_w


def integrate_rays(ray_points: np.ndarray, ray_dirs: np.ndarray, grid_min, shape, tsdf, weights, args):
    origin = np.asarray(args.sensor_origin, dtype=np.float64)

    n = len(ray_points)
    touched = 0
    t0 = time.time()

    for i in range(n):
        touched += integrate_one_ray(
            point=ray_points[i],
            ray_dir=ray_dirs[i],
            origin=origin,
            grid_min=grid_min,
            shape=shape,
            voxel_size=args.voxel_size,
            truncation=args.truncation,
            tsdf=tsdf,
            weights=weights,
            args=args,
        )

        if (i + 1) % args.progress_every == 0:
            elapsed = time.time() - t0
            print(f"Integrated {i+1:,}/{n:,} rays | touched {touched:,} voxels | {elapsed:.1f}s")

    return touched


# mesh

def extract_mesh(tsdf: np.ndarray, weights: np.ndarray, grid_min: np.ndarray, shape: np.ndarray, args):
    volume = tsdf.reshape(tuple(shape))

    wvol = weights.reshape(tuple(shape))
    volume = volume.copy()
    volume[wvol <= 0] = 1.0

    print("Running marching cubes...")
    verts, faces, normals, _values = measure.marching_cubes(
        volume,
        level=0.0,
        spacing=(args.voxel_size, args.voxel_size, args.voxel_size),
        allow_degenerate=False,
    )

    verts = verts + grid_min[None, :]

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    mesh.compute_vertex_normals()

    print(f"Raw mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")

    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()

    if args.smooth_iterations > 0:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=args.smooth_iterations)
        mesh.compute_vertex_normals()

    if args.min_component_triangles > 0 and len(mesh.triangles) > 0:
        mesh = remove_small_triangle_components(mesh, args.min_component_triangles)

    mesh.compute_vertex_normals()
    mesh.paint_uniform_color((0.72, 0.72, 0.70))

    print(f"Clean mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles")
    return mesh


def remove_small_triangle_components(mesh: o3d.geometry.TriangleMesh, min_triangles: int):
    tri_clusters, cluster_n_triangles, _cluster_area = mesh.cluster_connected_triangles()
    tri_clusters = np.asarray(tri_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    remove = np.zeros(len(mesh.triangles), dtype=bool)
    for cid, count in enumerate(cluster_n_triangles):
        if count < min_triangles:
            remove |= tri_clusters == cid

    mesh.remove_triangles_by_mask(remove)
    mesh.remove_unreferenced_vertices()
    return mesh

# presets that a arbitraryly picked the values for, Ive defaulted ultra adn honestly have no idea how it would run on a low end computer
def apply_detail_preset(args):
    if args.detail_preset is None:
        return args

    if args.detail_preset == "fast":
        args.voxel_size = 0.065
        args.truncation = 0.20
        args.max_rays = 120000
        args.surface_step = 0.04
        args.smooth_iterations = 1
    elif args.detail_preset == "balanced":
        args.voxel_size = 0.045
        args.truncation = 0.14
        args.max_rays = 220000
        args.surface_step = 0.025
        args.smooth_iterations = 2
    elif args.detail_preset == "high":
        args.voxel_size = 0.032
        args.truncation = 0.095
        args.max_rays = 420000
        args.surface_step = 0.018
        args.smooth_iterations = 1
    elif args.detail_preset == "ultra":
        args.voxel_size = 0.025
        args.truncation = 0.075
        args.max_rays = 700000
        args.surface_step = 0.0125
        args.smooth_iterations = 0

    return args

# pyvista hole filling was not working for me, the fill mesh hole needs tweaking per scan but if you get it right it works
def fill_mesh_holes_with_pyvista(mesh: o3d.geometry.TriangleMesh, hole_size: float) -> o3d.geometry.TriangleMesh:

    if hole_size <= 0 or len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        return mesh

    try:
        import pyvista as pv
    except Exception as e:
        print(f"Skipping hole fill: pyvista/vtk not available: {e}")
        return mesh

    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles, dtype=np.int64)

    faces = np.empty((len(tris), 4), dtype=np.int64)
    faces[:, 0] = 3
    faces[:, 1:] = tris

    poly = pv.PolyData(verts, faces.ravel())

    before_cells = poly.n_cells
    try:
        filled = poly.fill_holes(hole_size)
        filled = filled.triangulate()
        filled = filled.clean()
    except Exception as e:
        print(f"Hole fill failed: {e}")
        return mesh

    if filled.n_points == 0 or filled.n_cells == 0:
        print("Hole fill produced an empty mesh, keeping original")
        return mesh

    f = filled.faces.reshape((-1, 4))
    tris_out = f[:, 1:4].astype(np.int32)
    verts_out = np.asarray(filled.points, dtype=np.float64)

    out = o3d.geometry.TriangleMesh()
    out.vertices = o3d.utility.Vector3dVector(verts_out)
    out.triangles = o3d.utility.Vector3iVector(tris_out)
    out.remove_degenerate_triangles()
    out.remove_duplicated_triangles()
    out.remove_duplicated_vertices()
    out.compute_vertex_normals()
    out.paint_uniform_color((0.72, 0.72, 0.70))

    print(f"Hole fill: cells {before_cells:,} -> {len(out.triangles):,}")
    return out


def subdivide_for_visual_smoothness(mesh: o3d.geometry.TriangleMesh, iterations: int) -> o3d.geometry.TriangleMesh:
    if iterations <= 0 or len(mesh.triangles) == 0:
        return mesh

    for i in range(iterations):
        print(f"Subdividing mesh for visual smoothness: pass {i + 1}/{iterations}")
        mesh = mesh.subdivide_midpoint(number_of_iterations=1)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.compute_vertex_normals()

    mesh.paint_uniform_color((0.72, 0.72, 0.70))
    return mesh


def save_debug_outputs(outdir: Path, ray_points: np.ndarray, grid_min, shape, tsdf, weights, args):
    # save binned ray points as PLY for debug
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(ray_points)
    gray = np.full((len(ray_points), 3), 0.72, dtype=np.float64)
    pcd.colors = o3d.utility.Vector3dVector(gray)
    o3d.io.write_point_cloud(str(outdir / "binned_organized_rays.ply"), pcd, write_ascii=False)

    if args.save_tsdf_npz:
        np.savez_compressed(
            outdir / "tsdf_grid.npz",
            tsdf=tsdf.reshape(tuple(shape)).astype(np.float32),
            weights=weights.reshape(tuple(shape)).astype(np.float32),
            grid_min=grid_min.astype(np.float32),
            voxel_size=np.array([args.voxel_size], dtype=np.float32),
        )


# CLI
def parse_args():
    p = argparse.ArgumentParser(description="TSDF volumetric fusion from organized angular LiDAR range frames")

    p.add_argument("--input-csv", type=Path, default=DEFAULT_CSV)
    p.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    p.add_argument("--preview", action="store_true")

    p.add_argument("--sensor-origin", nargs=3, type=float, default=[0.0, 0.0, 0.0])

    # cleanin 
    p.add_argument("--min-range", type=float, default=0.12)
    p.add_argument("--max-range", type=float, default=8.0)
    p.add_argument("--clean-percentile", type=float, default=99.8)

    p.add_argument("--tilt-bin-deg", type=float, default=0.25)
    p.add_argument("--azimuth-bin-deg", type=float, default=0.35)
    p.add_argument("--bin-stat", choices=["median", "closest", "farthest"], default="median")
    p.add_argument("--fuse-segments-before-tsdf", action="store_true")

    p.add_argument("--max-rays", type=int, default=220000)
    p.add_argument("--random-seed", type=int, default=42)

    # TSDF
    p.add_argument("--detail-preset", choices=["fast", "balanced", "high", "ultra"], default=None,
                   help="")
    p.add_argument("--voxel-size", type=float, default=0.045)
    p.add_argument("--truncation", type=float, default=0.14)

    p.add_argument("--bounds", nargs=6, type=float, default=None, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    p.add_argument("--bounds-padding", type=float, default=0.25)
    p.add_argument("--bounds-percentile-low", type=float, default=0.5)
    p.add_argument("--bounds-percentile-high", type=float, default=99.5)
    p.add_argument("--max-voxels", type=int, default=18_000_000)

    # settings
    p.add_argument("--surface-step", type=float, default=0.025)
    p.add_argument("--surface-weight", type=float, default=1.0)
    p.add_argument("--max-weight", type=float, default=64.0)

    p.add_argument("--integrate-free-space", action="store_true")
    p.add_argument("--free-space-step", type=float, default=0.09)
    p.add_argument("--free-space-weight", type=float, default=0.10)

    # cleanup
    p.add_argument("--smooth-iterations", type=int, default=2)
    p.add_argument("--min-component-triangles", type=int, default=250)

    p.add_argument("--fill-holes-size", type=float, default=0.0,
                   help="")
    p.add_argument("--visual-subdivide", type=int, default=0,
                   help="")

    p.add_argument("--save-tsdf-npz", action="store_true")
    p.add_argument("--progress-every", type=int, default=10000)

    return p.parse_args()


def main():
    args = parse_args()
    args = apply_detail_preset(args)
    args.outdir.mkdir(parents=True, exist_ok=True)

    print("Loading CSV...")
    data_raw = load_scan_csv(args.input_csv)
    raw_rows = int(data_raw["raw_rows"][0])
    print(f"Raw CSV rows: {raw_rows:,}")

    data = filter_scan(data_raw, args)
    valid_rows = len(data["points"])
    print(f"Valid filtered rows: {valid_rows:,}")

    print("Building organized angular range frames...")
    ray_points, ray_dirs, organize_meta = build_organized_rays(data, args)
    print(json.dumps(organize_meta, indent=2))
    print(f"Rays to integrate: {len(ray_points):,}")

    if len(ray_points) == 0:
        raise RuntimeError("No rays to integrate")

    grid_min, shape, tsdf, weights = make_grid(ray_points, args)
    grid_max = grid_min + shape * args.voxel_size
    print(f"Grid min: {grid_min}")
    print(f"Grid max: {grid_max}")
    print(f"Grid shape: {shape.tolist()} = {int(np.prod(shape)):,} voxels")

    print("Integrating TSDF...")
    touched = integrate_rays(ray_points, ray_dirs, grid_min, shape, tsdf, weights, args)
    print(f"Total voxel updates: {touched:,}")
    print(f"Observed voxels: {int(np.sum(weights > 0)):,}")

    save_debug_outputs(args.outdir, ray_points, grid_min, shape, tsdf, weights, args)

    mesh = extract_mesh(tsdf, weights, grid_min, shape, args)

    if args.fill_holes_size > 0:
        mesh = fill_mesh_holes_with_pyvista(mesh, args.fill_holes_size)

    if args.visual_subdivide > 0:
        mesh = subdivide_for_visual_smoothness(mesh, args.visual_subdivide)

    mesh_path = args.outdir / "tsdf_room_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_ascii=False)
    print(f"Saved mesh: {mesh_path}")

    report = TsdfReport(
        source_csv=str(args.input_csv),
        raw_rows=raw_rows,
        valid_rows=valid_rows,
        binned_rays=int(organize_meta.get("organized_bins", len(ray_points))),
        integrated_rays=int(len(ray_points)),
        voxel_size_m=float(args.voxel_size),
        truncation_m=float(args.truncation),
        grid_min=[float(v) for v in grid_min],
        grid_max=[float(v) for v in grid_max],
        grid_shape=[int(v) for v in shape],
        mesh_vertices=int(len(mesh.vertices)),
        mesh_triangles=int(len(mesh.triangles)),
        notes=[],
    )

    with (args.outdir / "tsdf_report.json").open("w") as f:
        json.dump(asdict(report), f, indent=2)

    if args.preview:
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        o3d.visualization.draw_geometries([mesh, axis])


if __name__ == "__main__":
    main()
