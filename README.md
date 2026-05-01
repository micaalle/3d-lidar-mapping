# 3D LiDAR Room Scanner

**In progress**

This project is a custom-built 3D room scanning system using an **RPLIDAR A2** mounted on a motorized tilt axis.

The scanner converts 2D LiDAR sweeps into 3D point clouds by rotating the LiDAR on a second axis while recording the LiDAR scan angle, tilt angle, and distance measurements. The result is a 3D room scan that can be viewed as raw LiDAR points and compared against a generated mesh using a custom TSDF reconstruction pipeline.

---

## Video Demo

https://github.com/user-attachments/assets/63709ae9-6882-4be2-960b-7bfebc5fa824

> Note: The blob in the middle of the scan and the empty region beneath it are caused by noise from the current physical rig. I am currently working on reducing this

---

## Raw LiDAR Point Cloud

The raw point cloud shows the actual LiDAR measurements directly. 

<img width="2556" height="1365" src="https://github.com/user-attachments/assets/ef0ce074-f0a5-45da-92a4-2e511976c7c2" />

---

## TSDF Mesh Reconstruction

The mesh is generated from the captured LiDAR data using a TSDF-based reconstruction process. This helps visualize larger surfaces more clearly, although the mesh can mess up some finer details and teh process needs tuning but in its current state provides good supplementary context to the points.

<img width="2559" height="1369" src="https://github.com/user-attachments/assets/50393fa9-938b-4d76-a203-43aede121b27" />

---

## LiDAR Rig

<table>
  <tr>
    <td align="center">
      <strong>Original CAD model</strong><br>
      <img src="https://github.com/user-attachments/assets/2464ad8e-b35d-40b3-812e-9bb45266f2b8" width="1500" />
    </td>
    <td align="center">
      <strong>First build</strong><br>
      <img src="https://github.com/user-attachments/assets/6af3a0e2-b5b3-4cd9-9368-22020fc7c3b8" width="1500" />
    </td>
  </tr>
</table>

The current code is designed specifically for my scanner rig. The rig uses a **NEMA 17 stepper motor** connected to a **DRV8825 stepper driver** to rotate the LiDAR on a second axis, separate from the LiDAR’s internal spinning axis.

An **Arduino Nano ESP32** controls the motor movement, while an **AS5600 magnetic encoder** tracks the actual tilt angle of the LiDAR during the scan.

### Current Hardware

- RPLIDAR A2
- NEMA 17 stepper motor
- DRV8825 stepper driver
- Arduino Nano ESP32
- AS5600 magnetic encoder

---

## How to Run

This project has two main parts:

1. The Python scan / reconstruction pipeline
2. The C++ OpenGL viewer

The scanner produces the raw scan files, and the C++ viewer loads those files for interactive visualization. The viewer can also optionally run the TSDF mesh generation step on startup.

### 1. Create or copy the scan output

First, make sure the scan data exists inside the project folder.

Expected structure:

```text
<PROJECT_ROOT>/
  scan_output/
    room_scan.csv
    room_scan_raw.ply
    room_scan_filtered.ply
```

all you really need is room_scan.csv

```text
scan_output/room_scan.csv
```

This is the file the OpenGL viewer loads to display the LiDAR points.

---

### 2. Install the Python TSDF requirements

The mesh generation step uses Python.

From the main project folder, run:

```powershell
cd <PROJECT_ROOT>
pip install -r .\tsdf_range_fusion\requirements.txt
```

### 3. Make sure the external C++ libraries are present

The OpenGL viewer expects the external libraries to be inside the `external/` folder.

Expected structure:

```text
<PROJECT_ROOT>/
  external/
    glad/
      include/
      src/
        glad.c

    glfw-3.4/
      CMakeLists.txt
      include/
      src/

    glm/
      glm/

    imgui/
      imgui.cpp
      imgui_draw.cpp
      imgui_tables.cpp
      imgui_widgets.cpp
      backends/
        imgui_impl_glfw.cpp
        imgui_impl_glfw.h
        imgui_impl_opengl3.cpp
        imgui_impl_opengl3.h
```

### 4. Create the CMake build folder

From the main project folder:

```powershell
cd <PROJECT_ROOT>
mkdir build
cd build
```

Run:

```powershell
cmake ..
cmake --build . --config Release
```

### 5. Run the viewer

From inside the `build/` folder, run:

```powershell
.\Release\LidarRoomViewer.exe --root <PROJECT_ROOT>
```

The `--root` path should point to the folder that contains `scan_output/`.

So this file should exist:

```text
<PROJECT_ROOT>/scan_output/room_scan.csv
```

---

### 6. Choose whether to generate a mesh

When the viewer starts, it asks:

```text
Generate/update TSDF mesh now? [Y/N]:
```

To skip mesh generation and only view the raw LiDAR points:

```text
N
```

To generate or update the mesh before opening the viewer:

```text
Y
```

If you choose `Y`, it asks for a quality preset:

```text
1) ultra
2) high
3) balanced
4) fast
```

Pressing Enter defaults to:

```text
ultra
```

The generated mesh is saved to:

```text
tsdf_output/
  tsdf_room_mesh.ply
```

The viewer loads this mesh from:

```text
<PROJECT_ROOT>/tsdf_output/tsdf_room_mesh.ply
```

## Notes

The viewer is designed around the output format from my scanner. It expects a CSV file containing the reconstructed 3D points from the LiDAR scan.

## Current issues

The current rig setup has it going [(0, 360), (360, 0)] this creates two passes of scans and is mostly done like this because I do not currently have a slip ring setup. For the next version I would like to implement this so there are less issues with the wiring of the setup and it would just let me set it doing as many loops as needed rather then hardcoding a setpath to avoid wire wrap.

Another issue would be the TSDF mesh not being as clear as it could be, this just comes down to finer tuning adn giving it more data, if I was to give it another csv with lidar data from a different angle it would help fill in some of the hoels and make the overall implementation cleaner. However I would need to update my rig to be able to track movement relative to the room so the scans would line up appropriately. 

