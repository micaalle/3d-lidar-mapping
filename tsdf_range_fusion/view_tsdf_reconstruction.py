from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import open3d as o3d


DEFAULT_OUTDIR = Path("tsdf_output")


def load_mesh(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}. Run tsdf_fuse_organized.py first.")
    mesh = o3d.io.read_triangle_mesh(str(path))
    mesh.compute_vertex_normals()
    return mesh


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    p.add_argument("--raw", action="store_true", help="Overlay binned organized ray points")
    return p.parse_args()


def main():
    args = parse_args()

    mesh = load_mesh(args.outdir / "tsdf_room_mesh.ply")
    geoms = [mesh]

    if args.raw:
        pts_path = args.outdir / "binned_organized_rays.ply"
        if pts_path.exists():
            pcd = o3d.io.read_point_cloud(str(pts_path))
            if len(pcd.points):
                geoms.append(pcd)

    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))

    print("Open3D controls: left drag orbit, scroll/right drag zoom, shift+drag pan.")
    o3d.visualization.draw_geometries(geoms)


if __name__ == "__main__":
    main()
