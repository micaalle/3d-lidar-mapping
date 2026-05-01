from pathlib import Path
import json
import open3d as o3d

OUTDIR = Path("tsdf_output")

report_path = OUTDIR / "tsdf_report.json"
mesh_path = OUTDIR / "tsdf_room_mesh.ply"
rays_path = OUTDIR / "binned_organized_rays.ply"

if report_path.exists():
    report = json.loads(report_path.read_text())
    print(json.dumps(report, indent=2))
else:
    print("No tsdf_report.json found")

if mesh_path.exists():
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    print(f"mesh vertices: {len(mesh.vertices)}")
    print(f"mesh triangles: {len(mesh.triangles)}")
else:
    print("No tsdf_room_mesh.ply found")

if rays_path.exists():
    pcd = o3d.io.read_point_cloud(str(rays_path))
    print(f"binned rays: {len(pcd.points)}")
else:
    print("No binned_organized_rays.ply found")
