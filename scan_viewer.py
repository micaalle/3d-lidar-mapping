from pathlib import Path

import numpy as np
import open3d as o3d
import pyvista as pv

# either flitered or raw
DEFAULT_VIEW = "filtered"  

WINDOW_SIZE = (1600, 1000)
BACKGROUND = "#050608"
SHOW_AXES = True
POINT_SIZE = 4
USE_EDL = True
CAMERA_PRESET = "iso"

# grey scale 
GRAY_VALUE = 215   # 0-255


def get_scan_output_dir() -> Path:
    return Path(__file__).resolve().parent / "scan_output"


def choose_ply_file(scan_output_dir: Path) -> Path:
    raw_path = scan_output_dir / "room_scan_raw.ply"
    filtered_path = scan_output_dir / "room_scan_filtered.ply"

    if DEFAULT_VIEW == "raw":
        if raw_path.exists():
            return raw_path
        if filtered_path.exists():
            return filtered_path
    else:
        if filtered_path.exists():
            return filtered_path
        if raw_path.exists():
            return raw_path

    raise FileNotFoundError(
        f"Could not find room_scan_filtered.ply or room_scan_raw.ply in {scan_output_dir}"
    )


def load_point_cloud(ply_path: Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(ply_path))
    if len(pcd.points) == 0:
        raise RuntimeError(f"No points found in {ply_path}")
    return pcd


def setup_camera(plotter: pv.Plotter):
    if CAMERA_PRESET == "iso":
        plotter.camera_position = "iso"
        plotter.camera.zoom(1.2)
    elif CAMERA_PRESET == "xy":
        plotter.view_xy()
    elif CAMERA_PRESET == "xz":
        plotter.view_xz()
    elif CAMERA_PRESET == "yz":
        plotter.view_yz()


def main():
    scan_output_dir = get_scan_output_dir()
    ply_path = choose_ply_file(scan_output_dir)
    pcd = load_point_cloud(ply_path)

    points = np.asarray(pcd.points, dtype=np.float32)
    colors = np.full((len(points), 3), GRAY_VALUE, dtype=np.uint8)

    print(f"Loaded: {ply_path}")

    cloud = pv.PolyData(points)
    cloud["rgb"] = colors

    plotter = pv.Plotter(window_size=WINDOW_SIZE)
    plotter.set_background(BACKGROUND)

    plotter.add_points(
        cloud,
        scalars="rgb",
        rgb=True,
        render_points_as_spheres=True,
        point_size=POINT_SIZE,
        ambient=0.20,
        diffuse=0.80,
        specular=0.05,
        specular_power=6.0,
    )

    if USE_EDL:
        try:
            plotter.enable_eye_dome_lighting()
        except Exception:
            pass

    if SHOW_AXES:
        plotter.add_axes()

    setup_camera(plotter)
    plotter.show()


if __name__ == "__main__":
    main()
