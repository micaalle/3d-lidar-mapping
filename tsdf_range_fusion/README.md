# TSDF Volumetric Fusion from Organized Range Frames

This is the serious reconstruction path for your scanner.

Your scanner is not a regular RGB-D camera. It is a spherical/range scanner:

```text
tilt angle + LiDAR azimuth + range -> 3D point
```

So this package does custom **ray-based TSDF fusion**:

```text
raw room_scan.csv
↓
bin into organized angular range frames
↓
integrate LiDAR rays into a TSDF voxel volume
↓
extract zero-crossing surface with marching cubes
↓
save mesh
```

## Install

From your project virtual environment:

```bash
pip install -r requirements.txt
```

## First run

Put these files in your main `Project/` folder next to `scan_output/`.

Then run:

```bash
python tsdf_fuse_organized.py --preview
```

This creates:

```text
tsdf_output/tsdf_room_mesh.ply
tsdf_output/binned_organized_rays.ply
tsdf_output/tsdf_report.json
```

View later:

```bash
python view_tsdf_reconstruction.py
```

Overlay the binned raw rays:

```bash
python view_tsdf_reconstruction.py --raw
```

## Recommended commands

Balanced first test:

```bash
python tsdf_fuse_organized.py --voxel-size 0.045 --truncation 0.14 --max-rays 220000 --preview
```

Faster rough test:

```bash
python tsdf_fuse_organized.py --voxel-size 0.065 --truncation 0.20 --max-rays 100000 --preview
```

Higher detail:

```bash
python tsdf_fuse_organized.py --voxel-size 0.035 --truncation 0.10 --max-rays 400000 --preview
```

If the mesh is too blobby:

```bash
python tsdf_fuse_organized.py --voxel-size 0.035 --truncation 0.08 --max-rays 350000 --smooth-iterations 0 --preview
```

If the mesh has holes:

```bash
python tsdf_fuse_organized.py --voxel-size 0.05 --truncation 0.18 --max-rays 350000 --preview
```

If there are lots of floating fragments:

```bash
python tsdf_fuse_organized.py --min-component-triangles 1000 --preview
```

## Bounds

If the grid is too big, manually bound the room:

```bash
python tsdf_fuse_organized.py --bounds -4 4 -2 3 -4 4 --preview
```

Format:

```text
--bounds xmin xmax ymin ymax zmin zmax
```

For your current scan, visual height often seems to be `Y`, but TSDF does not care about height labels. It works in raw XYZ.

## Important notes

TSDF works best when:
- calibration is good
- repeated views overlap consistently
- the stage motion is smooth
- the input cloud has enough density

If the scan still has motion distortion, TSDF will fuse that distortion too. This is not a magic fix for bad geometry, but it is the serious reconstruction method.


## v2: higher detail and hole filling

This version adds:

```bash
--detail-preset fast|balanced|high|ultra
--fill-holes-size <meters>
--visual-subdivide <iterations>
```

### More real detail

Use smaller voxels and more rays:

```bash
python tsdf_fuse_organized.py --detail-preset high --preview
```

or:

```bash
python tsdf_fuse_organized.py --detail-preset ultra --preview
```

The `ultra` preset can be slow and memory-heavy.

### Fill occlusion holes / shadow gaps

If the mesh has small empty shadow-like holes, try:

```bash
python tsdf_fuse_organized.py --detail-preset high --fill-holes-size 0.35 --preview
```

If holes remain:

```bash
python tsdf_fuse_organized.py --detail-preset high --fill-holes-size 0.65 --preview
```

Be careful: hole filling is a completion/visual step. It cannot recover true hidden geometry behind objects; it only closes mesh gaps.

### More polygons without true detail

This only smooths the visual mesh:

```bash
python tsdf_fuse_organized.py --detail-preset high --visual-subdivide 1 --preview
```

Real detail comes from `--voxel-size`, `--truncation`, and `--max-rays`, not subdivision.
