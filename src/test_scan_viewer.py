# pyvista tester for mesh


from pathlib import Path
import csv

import numpy as np
import open3d as o3d
import pyvista as pv

try:
    import vtk
except Exception:
    vtk = None


DEFAULT_VIEW = "raw"  

WINDOW_SIZE = (1600, 1000)
BACKGROUND_DARK = "#050608"
BACKGROUND_LIGHT = "white"
SHOW_AXES = True
POINT_SIZE = 4
USE_EDL = True
CAMERA_PRESET = "iso"


GRAY_VALUE = 215  


DEFAULT_RENDER_DISTANCE_PERCENTILE = 100.0
DEFAULT_PROGRESS_DEG = 360.0

MESH_OPACITY = 0.28
MESH_COLOR_DARK_BG = (0.58, 0.58, 0.56)
MESH_COLOR_LIGHT_BG = (0.42, 0.42, 0.40)


def get_project_dir() -> Path:
    return Path(__file__).resolve().parent


def get_scan_output_dir() -> Path:
    return get_project_dir() / "scan_output"


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


def choose_csv_file(scan_output_dir: Path) -> Path | None:
    csv_path = scan_output_dir / "room_scan.csv"
    return csv_path if csv_path.exists() else None


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


def load_csv_points_in_capture_order(csv_path: Path):
    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                rows.append(row)

    if not rows:
        raise RuntimeError(f"CSV is empty: {csv_path}")

    first = [c.strip().lower() for c in rows[0]]
    has_header = "x" in first and "y" in first and "z" in first

    points = []

    if has_header:
        x_i = first.index("x")
        y_i = first.index("y")
        z_i = first.index("z")
        data_rows = rows[1:]
    else:
        x_i, y_i, z_i = -3, -2, -1
        data_rows = rows

    for row in data_rows:
        try:
            x = float(row[x_i])
            y = float(row[y_i])
            z = float(row[z_i])
        except Exception:
            continue

        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            points.append([x, y, z])

    if not points:
        raise RuntimeError(f"No valid x,y,z points found in {csv_path}")

    return np.asarray(points, dtype=np.float32)


def find_mesh_file(project_dir: Path) -> Path | None:
    candidates = [
        project_dir / "tsdf_output" / "tsdf_room_mesh.ply",
    ]

    for path in candidates:
        if path.exists():
            return path

    return None


def load_mesh_for_pyvista(mesh_path: Path):
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError(f"Mesh has no triangles: {mesh_path}")

    mesh.compute_vertex_normals()

    verts = np.asarray(mesh.vertices, dtype=np.float32)
    tris = np.asarray(mesh.triangles, dtype=np.int64)

    faces = np.empty((len(tris), 4), dtype=np.int64)
    faces[:, 0] = 3
    faces[:, 1:] = tris

    return pv.PolyData(verts, faces.ravel())


def add_bottom_right_panel(plotter: pv.Plotter):
    if vtk is None:
        return None

    x0, y0 = 0.695, 0.035
    x1, y1 = 0.975, 0.205

    points = vtk.vtkPoints()
    points.InsertNextPoint(x0, y0, 0.0)
    points.InsertNextPoint(x1, y0, 0.0)
    points.InsertNextPoint(x1, y1, 0.0)
    points.InsertNextPoint(x0, y1, 0.0)

    polygon = vtk.vtkPolygon()
    polygon.GetPointIds().SetNumberOfIds(4)
    for i in range(4):
        polygon.GetPointIds().SetId(i, i)

    cells = vtk.vtkCellArray()
    cells.InsertNextCell(polygon)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(cells)

    coord = vtk.vtkCoordinate()
    coord.SetCoordinateSystemToNormalizedViewport()

    mapper = vtk.vtkPolyDataMapper2D()
    mapper.SetInputData(polydata)
    mapper.SetTransformCoordinate(coord)

    actor = vtk.vtkActor2D()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.18, 0.18, 0.18)
    actor.GetProperty().SetOpacity(0.82)

    plotter.renderer.AddActor2D(actor)
    return actor


class ScanViewer:
    def __init__(self):
        self.project_dir = get_project_dir()
        self.scan_output_dir = get_scan_output_dir()

        self.ply_path = choose_ply_file(self.scan_output_dir)
        self.csv_path = choose_csv_file(self.scan_output_dir)

        pcd = load_point_cloud(self.ply_path)
        self.points = np.asarray(pcd.points, dtype=np.float32)

        if self.csv_path is not None:
            try:
                csv_points = load_csv_points_in_capture_order(self.csv_path)
                if len(csv_points) > 0:
                    self.points_for_progress = csv_points
                    print(f"Loaded CSV for progress order: {self.csv_path}")
                else:
                    self.points_for_progress = self.points
            except Exception as e:
                print(f"Could not use CSV for progress order: {e}")
                self.points_for_progress = self.points
        else:
            self.points_for_progress = self.points

        self.radius = np.linalg.norm(self.points_for_progress, axis=1)
        self.max_render_distance = float(np.percentile(self.radius, DEFAULT_RENDER_DISTANCE_PERCENTILE))
        self.min_render_distance = float(np.min(self.radius))

        self.colors = np.full((len(self.points_for_progress), 3), GRAY_VALUE, dtype=np.uint8)

        self.mesh_path = find_mesh_file(self.project_dir)
        self.mesh_poly = None
        if self.mesh_path is not None:
            try:
                self.mesh_poly = load_mesh_for_pyvista(self.mesh_path)
                print(f"Mesh available for M toggle: {self.mesh_path}")
            except Exception as e:
                print(f"Found mesh but could not load it: {self.mesh_path}")
                print(e)
                self.mesh_poly = None
        else:
            print("No mesh found for M toggle.")

        self.plotter = pv.Plotter(window_size=WINDOW_SIZE)

        self.background_is_dark = True
        self.current_render_distance = self.max_render_distance
        self.current_progress_deg = DEFAULT_PROGRESS_DEG

        self.points_actor = None
        self.mesh_actor = None
        self.text_actor = None
        self.panel_actor = None

        self.mesh_visible = False

    def current_background(self):
        return BACKGROUND_DARK if self.background_is_dark else BACKGROUND_LIGHT

    def current_mesh_color(self):
        return MESH_COLOR_DARK_BG if self.background_is_dark else MESH_COLOR_LIGHT_BG

    def current_text_color(self):
        return "white" if self.background_is_dark else "black"

    def get_visible_points(self):
        progress_fraction = np.clip(self.current_progress_deg / 360.0, 0.0, 1.0)
        progress_count = int(progress_fraction * len(self.points_for_progress))

        if progress_count <= 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        pts = self.points_for_progress[:progress_count]
        cols = self.colors[:progress_count]

        dist = self.radius[:progress_count]
        keep = dist <= self.current_render_distance

        return pts[keep], cols[keep]

    def redraw_all(self):
        self.plotter.set_background(self.current_background())

        if self.mesh_actor is not None:
            self.plotter.remove_actor(self.mesh_actor)
            self.mesh_actor = None

        if self.points_actor is not None:
            self.plotter.remove_actor(self.points_actor)
            self.points_actor = None

        if self.mesh_visible and self.mesh_poly is not None:
            self.mesh_actor = self.plotter.add_mesh(
                self.mesh_poly,
                color=self.current_mesh_color(),
                opacity=MESH_OPACITY,
                smooth_shading=True,
                specular=0.08,
                roughness=0.80,
            )

        visible_points, visible_colors = self.get_visible_points()
        cloud = pv.PolyData(visible_points)
        cloud["rgb"] = visible_colors

        self.points_actor = self.plotter.add_points(
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

        self.update_text(len(visible_points))
        self.plotter.render()

    def redraw_points_only(self):
        if self.points_actor is not None:
            self.plotter.remove_actor(self.points_actor)
            self.points_actor = None

        visible_points, visible_colors = self.get_visible_points()
        cloud = pv.PolyData(visible_points)
        cloud["rgb"] = visible_colors

        self.points_actor = self.plotter.add_points(
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

        self.update_text(len(visible_points))
        self.plotter.render()

    def update_text(self, visible_count: int):
        if self.text_actor is not None:
            self.plotter.remove_actor(self.text_actor)
            self.text_actor = None

        mesh_status = "ON" if self.mesh_visible else "OFF"
        mesh_name = self.mesh_path.name if self.mesh_path is not None else "none"

        text = (
            f"Loaded: {self.ply_path.name} | visible: {visible_count:,}/{len(self.points_for_progress):,}\n"
            f"Distance: {self.current_render_distance:.2f} m | Progress: {self.current_progress_deg:.1f}/360 deg | Mesh: {mesh_status} ({mesh_name})\n"
            "M mesh | B background | R reset | Q quit"
        )

        self.text_actor = self.plotter.add_text(
            text,
            position="upper_left",
            font_size=9,
            color=self.current_text_color(),
        )

    def on_render_distance_slider(self, value):
        self.current_render_distance = float(value)
        self.redraw_points_only()

    def on_progress_slider(self, value):
        self.current_progress_deg = float(value)
        self.redraw_points_only()

    def toggle_mesh(self):
        if self.mesh_poly is None:
            print("No mesh file was found. Run a TSDF/reconstruction script first.")
            return

        self.mesh_visible = not self.mesh_visible
        self.redraw_all()

    def toggle_background(self):
        self.background_is_dark = not self.background_is_dark
        self.redraw_all()

    def setup_sliders(self):
        self.panel_actor = add_bottom_right_panel(self.plotter)


        self.plotter.add_slider_widget(
            callback=self.on_render_distance_slider,
            rng=[self.min_render_distance, self.max_render_distance],
            value=self.current_render_distance,
            title="Distance",
            pointa=(0.725, 0.080),
            pointb=(0.955, 0.080),
            style="modern",
            title_height=0.018,
            slider_width=0.012,
            tube_width=0.006,
            fmt="%.2f",
        )

        self.plotter.add_slider_widget(
            callback=self.on_progress_slider,
            rng=[0.0, 360.0],
            value=self.current_progress_deg,
            title="Progress",
            pointa=(0.725, 0.155),
            pointb=(0.955, 0.155),
            style="modern",
            title_height=0.018,
            slider_width=0.012,
            tube_width=0.006,
            fmt="%.0f°",
        )

    def setup_keys(self):
        self.plotter.add_key_event("m", self.toggle_mesh)
        self.plotter.add_key_event("M", self.toggle_mesh)
        self.plotter.add_key_event("b", self.toggle_background)
        self.plotter.add_key_event("B", self.toggle_background)

    def run(self):
        print(f"Loaded: {self.ply_path}")
        if self.csv_path is not None:
            print(f"CSV progress source: {self.csv_path}")
        else:
            print("No room_scan.csv found; progress slider uses PLY point order.")

        self.plotter.set_background(self.current_background())

        self.redraw_all()

        if USE_EDL:
            try:
                self.plotter.enable_eye_dome_lighting()
            except Exception:
                pass

        if SHOW_AXES:
            self.plotter.add_axes()

        self.setup_sliders()
        self.setup_keys()
        setup_camera(self.plotter)

        self.plotter.show()


def main():
    viewer = ScanViewer()
    viewer.run()


if __name__ == "__main__":
    main()
