import csv
import math
import queue
import re
import subprocess
import threading
import time
import termios
from bisect import bisect_left
from pathlib import Path

import numpy as np
import open3d as o3d
import serial
from serial.tools import list_ports

# my arduino hardcoded and ports

ARDUINO_BAUD = 115200
ARDUINO_PORT = None
ARDUINO_VID = 0x2341
ARDUINO_PID = 0x0070
ARDUINO_SERIAL = "206EF1332804"

LIDAR_PORT = "/dev/cu.usbserial-0001"
LIDAR_BAUD = 115200

ULTRA_SIMPLE_PATH = str(
    Path.home() / "rplidar_sdk" / "output" / "Darwin" / "Release" / "ultra_simple"
)

# Keep False unless HOME is fully verified and safe
USE_HOME_AT_START = False

SWEEP_SPEED_DPS = 4.0

# avoid going 360 have end point not be past -85 to 85
SWEEP_SEGMENTS = [(-95.0, 85.0), (85.0, -95.0), (-95.0, 85.0)]

PRE_SWEEP_SETTLE_SEC = 0.6
POST_SWEEP_SETTLE_SEC = 0.2

LOCAL_BEAM_AXIS = "z"
SCAN_ROT_AXIS = "x"
TILT_ROT_AXIS = "y"

SCAN_SIGN = -1.0
TILT_SIGN = 1.0
SCAN_ZERO_OFFSET_DEG = 0.0
TILT_ZERO_OFFSET_DEG = 0.0

SENSOR_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float64)





MIN_DISTANCE_M = 0.15
MAX_DISTANCE_M = 8.0
MIN_QUALITY = 0

VOXEL_SIZE = 0.03
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0

SHOW_VIEWER = False

OUTDIR = Path("scan_output")
OUTDIR.mkdir(exist_ok=True)




def wrap180(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def print_ports():
    print("Detected serial ports:")
    for p in sorted(list_ports.comports()):
        print(f"  {p.device} | {p.description} | {p.hwid}")
    print()


def find_arduino_port():
    matches = []
    for p in list_ports.comports():
        desc = p.description or ""
        serial_num = getattr(p, "serial_number", None)

        if p.vid == ARDUINO_VID and p.pid == ARDUINO_PID:
            if ARDUINO_SERIAL is None or serial_num == ARDUINO_SERIAL:
                matches.append(p.device)
        elif "Nano ESP32" in desc:
            matches.append(p.device)

    if not matches:
        raise RuntimeError("Could not find Nano ESP32 serial port")

    return matches[0]


def arduino_open(port: str | None = None) -> serial.Serial:
    if port is None:
        port = find_arduino_port()

    print(f"Opening Arduino on {port}")
    ser = serial.Serial(port, ARDUINO_BAUD, timeout=0.02)
    time.sleep(3.0)

    deadline = time.time() + 2.0
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("ARDUINO:", line)
        else:
            time.sleep(0.05)

    return ser


def reconnect_arduino():
    print("Reconnecting Arduino...")
    deadline = time.time() + 15.0
    last_error = None

    while time.time() < deadline:
        try:
            return arduino_open(None)
        except Exception as e:
            last_error = e
            time.sleep(0.5)

    raise RuntimeError(f"Failed to reconnect Arduino within timeout: {last_error}")


def arduino_cmd(
    ser: serial.Serial,
    cmd: str,
    retries: int = 1,
    response_timeout: float = 2.0,
):
    last_error = None

    for i in range(retries):
        try:
            print(f"DEBUG: sending {cmd}, try {i+1}/{retries}")

            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            ser.write((cmd.strip() + "\n").encode())
            ser.flush()

            t0 = time.time()
            while time.time() - t0 < response_timeout:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"ARDUINO << {line}")
                    if line.startswith("ERR"):
                        raise RuntimeError(f"Arduino error for {cmd}: {line}")
                    return line, ser
                time.sleep(0.005)

            time.sleep(0.2)

        except (serial.SerialException, OSError, termios.error) as e:
            last_error = e
            print(f"Arduino serial error: {e}")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)
            ser = reconnect_arduino()

            line, ser = arduino_cmd(ser, "PING", retries=1, response_timeout=3.0)
            print(line)
            line, ser = arduino_cmd(ser, "STATUS", retries=1, response_timeout=3.0)
            print(line)

    raise RuntimeError(f"No response for command: {cmd}. Last error: {last_error}")


def goto_angle_blocking(
    arduino,
    target_deg: float,
    acceptable_deg: float = 1.0,
    timeout: float = 40.0,
):
    try:
        resp, arduino = arduino_cmd(
            arduino,
            f"GOTO {target_deg:.3f}",
            retries=1,
            response_timeout=timeout,
        )
        print(resp)
        return arduino
    except RuntimeError as e:
        if "ERR DIVERGING" not in str(e):
            raise

        resp, arduino = arduino_cmd(arduino, "ANGLE?", retries=1, response_timeout=3.0)
        print(resp)
        current_deg = float(resp.split()[-1])
        err = abs(wrap180(target_deg - current_deg))

        if err <= acceptable_deg:
            print(
                f"Accepted near target: current={current_deg:.3f}, "
                f"target={target_deg:.3f}, err={err:.3f}"
            )
            return arduino

        raise


def get_current_angle(arduino):
    resp, arduino = arduino_cmd(arduino, "ANGLE?", retries=1, response_timeout=3.0)
    print(resp)
    current_deg = float(resp.split()[-1])
    return arduino, current_deg


def sweep_return_to_zero(arduino, speed_dps: float = 4.0):
    arduino, current_deg = get_current_angle(arduino)

    print(f"\nReturning to 0.0 deg by sweep from {current_deg:.3f} deg...")

    resp, arduino = arduino_cmd(
        arduino,
        f"SWEEP {current_deg:.3f} 0.000 {speed_dps:.3f}",
        retries=1,
        response_timeout=3.0,
    )
    print(resp)

    deadline = time.time() + (abs(current_deg) / speed_dps) + 10.0
    while time.time() < deadline:
        line = arduino.readline().decode(errors="ignore").strip()
        if line:
            print(f"ARDUINO << {line}")
            if line.startswith("INFO,SWEEP_DONE"):
                return arduino
        time.sleep(0.002)

    raise RuntimeError("Timed out waiting for final sweep return to zero.")


_LINE_RE = re.compile(
    r"^\s*(S)?\s*theta:\s*([0-9.]+)\s+Dist:\s*([0-9.]+)\s+Q:\s*([0-9]+)"
)

class UltraSimpleReader:
    def __init__(self, ultra_simple_path: str, lidar_port: str, baudrate: int):
        self.cmd = [
            ultra_simple_path,
            "--channel",
            "--serial",
            lidar_port,
            str(baudrate),
        ]
        self.proc = None
        self.thread = None
        self.queue = queue.Queue()
        self.stop_flag = threading.Event()

    def start(self):
        self.proc = subprocess.Popen(
            self.cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        time.sleep(1.0)

    def _reader_loop(self):
        try:
            while not self.stop_flag.is_set():
                if self.proc is None or self.proc.stdout is None:
                    break

                line = self.proc.stdout.readline()
                t_ns = time.monotonic_ns()

                if not line:
                    if self.proc.poll() is not None:
                        break
                    continue

                m = _LINE_RE.match(line.strip())
                if not m:
                    continue

                start_flag = m.group(1) is not None
                angle_deg = float(m.group(2))
                distance_m = float(m.group(3)) / 1000.0
                quality = int(m.group(4))

                self.queue.put((t_ns, start_flag, angle_deg, distance_m, quality))
        except Exception as e:
            self.queue.put(("__ERROR__", str(e)))

    def clear_queue(self):
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break

    def drain_available(self):
        items = []
        while True:
            try:
                items.append(self.queue.get_nowait())
            except queue.Empty:
                break
        return items

    def stop(self):
        self.stop_flag.set()
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass




def axis_vector(axis: str) -> np.ndarray:
    axis = axis.lower()
    if axis == "x":
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if axis == "y":
        return np.array([0.0, 1.0, 0.0], dtype=np.float64)
    if axis == "z":
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    raise ValueError("Axis must be 'x', 'y', or 'z'")


def rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    a = math.radians(angle_deg)
    c = math.cos(a)
    s = math.sin(a)
    axis = axis.lower()

    if axis == "x":
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s,  c],
        ], dtype=np.float64)

    if axis == "y":
        return np.array([
            [ c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ], dtype=np.float64)

    if axis == "z":
        return np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    raise ValueError("Axis must be 'x', 'y', or 'z'")


def lidar_sample_to_xyz(distance_m: float, azimuth_deg: float, tilt_deg: float):
    local_beam = axis_vector(LOCAL_BEAM_AXIS) * distance_m
    scan_angle = SCAN_SIGN * (azimuth_deg + SCAN_ZERO_OFFSET_DEG)
    scan_R = rotation_matrix(SCAN_ROT_AXIS, scan_angle)
    tilt_angle = TILT_SIGN * (tilt_deg + TILT_ZERO_OFFSET_DEG)
    tilt_R = rotation_matrix(TILT_ROT_AXIS, tilt_angle)
    p = tilt_R @ (scan_R @ local_beam + SENSOR_OFFSET)
    return float(p[0]), float(p[1]), float(p[2])



# flitering 

def keep_sample(quality: int, distance_m: float) -> bool:
    if distance_m <= 0:
        return False
    if distance_m < MIN_DISTANCE_M or distance_m > MAX_DISTANCE_M:
        return False
    if quality < MIN_QUALITY:
        return False
    return True


def make_terrain_colors(points_xyz: np.ndarray) -> np.ndarray:
    if len(points_xyz) == 0:
        return np.empty((0, 3), dtype=np.float64)

    z = points_xyz[:, 2]
    z_min = float(np.min(z))
    z_max = float(np.max(z))
    t = (z - z_min) / (z_max - z_min + 1e-9)

    stops = np.array([
        [0.08, 0.12, 0.20],
        [0.18, 0.28, 0.35],
        [0.18, 0.42, 0.40],
        [0.40, 0.46, 0.28],
        [0.62, 0.54, 0.38],
        [0.82, 0.76, 0.62],
    ], dtype=np.float64)

    pos = np.linspace(0.0, 1.0, len(stops))
    colors = np.zeros((len(points_xyz), 3), dtype=np.float64)

    for c in range(3):
        colors[:, c] = np.interp(t, pos, stops[:, c])

    return colors


def build_filtered_point_cloud(points_xyz: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)

    pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB_NEIGHBORS,
        std_ratio=OUTLIER_STD_RATIO
    )

    filtered_pts = np.asarray(pcd.points)
    colors = make_terrain_colors(filtered_pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# sweep

def interpolate_tilt(t_ns: int, enc_times: list[int], enc_angles: list[float]):
    if len(enc_times) < 2:
        return None
    if t_ns < enc_times[0] or t_ns > enc_times[-1]:
        return None

    idx = bisect_left(enc_times, t_ns)
    if idx <= 0:
        return enc_angles[0]
    if idx >= len(enc_times):
        return enc_angles[-1]

    t0 = enc_times[idx - 1]
    t1 = enc_times[idx]
    a0 = enc_angles[idx - 1]
    a1 = enc_angles[idx]

    if t1 == t0:
        return a0

    alpha = (t_ns - t0) / (t1 - t0)
    return a0 + alpha * (a1 - a0)


def run_sweep(
    arduino,
    sdk_reader: UltraSimpleReader,
    start_deg: float,
    end_deg: float,
    speed_dps: float,
):
    arduino = goto_angle_blocking(arduino, start_deg, acceptable_deg=1.0, timeout=40.0)

    print(f"Settling at start angle {start_deg:.2f} deg...")
    time.sleep(PRE_SWEEP_SETTLE_SEC)

    sdk_reader.clear_queue()
    try:
        arduino.reset_input_buffer()
    except Exception:
        pass

    resp, arduino = arduino_cmd(
        arduino,
        f"SWEEP {start_deg:.3f} {end_deg:.3f} {speed_dps:.3f}",
        retries=1,
        response_timeout=3.0,
    )
    print(resp)

    lidar_samples = []
    encoder_samples = []
    saw_done = False
    done_reason = None
    deadline = time.time() + (abs(end_deg - start_deg) / speed_dps) + 10.0

    while time.time() < deadline:
        for item in sdk_reader.drain_available():
            if len(item) == 2 and item[0] == "__ERROR__":
                raise RuntimeError(f"SDK reader thread error: {item[1]}")
            t_ns, start_flag, angle_deg, distance_m, quality = item
            lidar_samples.append((t_ns, angle_deg, distance_m, quality, start_flag))

        line = arduino.readline().decode(errors="ignore").strip()
        if line:
            host_t_ns = time.monotonic_ns()

            if line.startswith("ANG,"):
                parts = line.split(",")
                if len(parts) >= 3:
                    angle_deg = float(parts[2])
                    encoder_samples.append((host_t_ns, angle_deg))
            elif line.startswith("INFO,SWEEP_DONE"):
                print(f"ARDUINO << {line}")
                saw_done = True
                parts = line.split(",")
                if len(parts) >= 5:
                    done_reason = parts[4]
                break
            else:
                print(f"ARDUINO << {line}")

        time.sleep(0.002)

    time.sleep(POST_SWEEP_SETTLE_SEC)
    for item in sdk_reader.drain_available():
        if len(item) == 2 and item[0] == "__ERROR__":
            raise RuntimeError(f"SDK reader thread error: {item[1]}")
        t_ns, start_flag, angle_deg, distance_m, quality = item
        lidar_samples.append((t_ns, angle_deg, distance_m, quality, start_flag))

    if not saw_done:
        raise RuntimeError("Sweep did not finish cleanly. No INFO,SWEEP_DONE received.")

    if done_reason not in ("OK", "STOP"):
        print(f"Warning: sweep finished with reason={done_reason}")

    print(f"Collected {len(lidar_samples)} LiDAR samples and {len(encoder_samples)} encoder samples")
    return arduino, lidar_samples, encoder_samples


def main():
    print_ports()

    arduino = arduino_open(ARDUINO_PORT)
    sdk_reader = UltraSimpleReader(ULTRA_SIMPLE_PATH, LIDAR_PORT, LIDAR_BAUD)
    sdk_reader.start()

    csv_path = OUTDIR / "room_scan.csv"
    raw_ply_path = OUTDIR / "room_scan_raw.ply"
    filtered_ply_path = OUTDIR / "room_scan_filtered.ply"

    all_points = []
    all_rows = []

    try:
        resp, arduino = arduino_cmd(arduino, "PING", retries=1, response_timeout=3.0)
        print(resp)

        resp, arduino = arduino_cmd(arduino, "ENABLE", retries=1, response_timeout=3.0)
        print(resp)

        if USE_HOME_AT_START:
            resp, arduino = arduino_cmd(arduino, "HOME", retries=1, response_timeout=20.0)
            print(resp)
        else:
            print("Using ZEROHERE as the start reference.\n")
            time.sleep(1.0)
            resp, arduino = arduino_cmd(arduino, "ZEROHERE", retries=1, response_timeout=3.0)
            print(resp)

        for segment_id, (start_deg, end_deg) in enumerate(SWEEP_SEGMENTS):
            print(f"\n=== Sweep segment {segment_id}: {start_deg:.1f} -> {end_deg:.1f} deg ===")
            arduino, lidar_samples, encoder_samples = run_sweep(
                arduino, sdk_reader, start_deg, end_deg, SWEEP_SPEED_DPS
            )

            enc_times = [t for t, _ in encoder_samples]
            enc_angles = [a for _, a in encoder_samples]

            kept = 0
            for t_ns, azimuth_deg, distance_m, quality, start_flag in lidar_samples:
                if not keep_sample(quality, distance_m):
                    continue

                tilt_deg = interpolate_tilt(t_ns, enc_times, enc_angles)
                if tilt_deg is None:
                    continue

                x, y, z = lidar_sample_to_xyz(distance_m, azimuth_deg, tilt_deg)

                all_rows.append([
                    segment_id,
                    t_ns,
                    tilt_deg,
                    azimuth_deg,
                    distance_m,
                    quality,
                    int(start_flag),
                    x, y, z,
                ])
                all_points.append([x, y, z])
                kept += 1

            print(f"Mapped {kept} points for segment {segment_id}")

        if USE_HOME_AT_START:
            print("\nReturning home...")
            resp, arduino = arduino_cmd(arduino, "HOME", retries=1, response_timeout=20.0)
            print(resp)
        else:
            arduino = sweep_return_to_zero(arduino, speed_dps=SWEEP_SPEED_DPS)

        resp, arduino = arduino_cmd(arduino, "DISABLE", retries=1, response_timeout=3.0)
        print(resp)

        if not all_points:
            raise RuntimeError("No points captured.")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "segment_id",
                "host_timestamp_ns",
                "tilt_deg",
                "azimuth_deg",
                "distance_m",
                "quality",
                "scan_start_flag",
                "x", "y", "z",
            ])
            writer.writerows(all_rows)

        raw_pts = np.asarray(all_points, dtype=np.float64)

        raw_pcd = o3d.geometry.PointCloud()
        raw_pcd.points = o3d.utility.Vector3dVector(raw_pts)
        raw_pcd.colors = o3d.utility.Vector3dVector(make_terrain_colors(raw_pts))
        raw_ok = o3d.io.write_point_cloud(str(raw_ply_path), raw_pcd, write_ascii=False)

        filtered_pcd = build_filtered_point_cloud(raw_pts)
        filtered_ok = o3d.io.write_point_cloud(str(filtered_ply_path), filtered_pcd, write_ascii=False)

        print(f"\nCSV saved to: {csv_path}")
        print(f"Raw PLY saved to: {raw_ply_path} (ok={raw_ok})")
        print(f"Filtered PLY saved to: {filtered_ply_path} (ok={filtered_ok})")

        if SHOW_VIEWER:
            o3d.visualization.draw_geometries([filtered_pcd])

    finally:
        try:
            resp, arduino = arduino_cmd(arduino, "STOP", retries=1, response_timeout=3.0)
            print(resp)
        except Exception:
            pass

        try:
            sdk_reader.stop()
        except Exception:
            pass

        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
