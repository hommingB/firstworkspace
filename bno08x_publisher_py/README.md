# BNO085 ROS2 Publisher

A ROS2 node that reads IMU data from a **BNO085** 9-DOF sensor over I2C on a **Raspberry Pi 5** and publishes it for visualization in **RViz2** on a remote machine.

---

## Hardware Wiring (I2C mode)

| BNO085 Pin | Raspberry Pi 5 Pin | Notes            |
|------------|-------------------|------------------|
| VIN        | Pin 1 (3.3 V)     |                  |
| GND        | Pin 6 (GND)       |                  |
| SDA        | Pin 3 (GPIO 2)    |                  |
| SCL        | Pin 5 (GPIO 3)    |                  |
| PS0        | GND               | Selects I2C mode |
| PS1        | GND               | Address = 0x4A   |

> Tie PS1 HIGH (3.3 V) to get address **0x4B** and update `i2c_address` in the YAML.

Enable I2C on the Pi: `sudo raspi-config` → Interface Options → I2C → Enable.

Verify the sensor appears: `i2cdetect -y 1`  (should show `4a` or `4b`)

---

## Installation

### On the Raspberry Pi 5 (publisher)

```bash
# 1. ROS2 Jazzy (or Humble) – follow official install guide if not done
#    https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# 2. Python driver for BNO085
pip3 install adafruit-circuitpython-bno08x --break-system-packages

# 3. Clone / copy this package into your workspace
mkdir -p ~/ros2_ws/src
cp -r bno085_publisher ~/ros2_ws/src/

# 4. Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --packages-select bno085_publisher
source install/setup.bash
```

### On the remote visualization machine (RViz2)

```bash
# Install rviz_imu_plugin for the IMU display
sudo apt install ros-$ROS_DISTRO-rviz-imu-plugin
```

---

## Running

### Step 1 – Set up ROS_DOMAIN_ID (same on BOTH machines)

```bash
export ROS_DOMAIN_ID=42     # pick any number 0–232
```

Or add it to `~/.bashrc` on both machines.

### Step 2 – Start the node on the Pi

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bno085_publisher bno085.launch.py
```

Or run directly with custom params:
```bash
ros2 run bno085_publisher bno085_node \
  --ros-args -p publish_rate_hz:=100.0 -p frame_id:=imu_link
```

### Step 3 – Verify topics on the Pi

```bash
ros2 topic list
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

### Step 4 – Visualize on the remote machine

```bash
# Make sure both machines are on the same LAN and ROS_DOMAIN_ID matches
source /opt/ros/$ROS_DISTRO/setup.bash
export ROS_DOMAIN_ID=42

# Open RViz2 with the provided config
rviz2 -d ~/ros2_ws/src/bno085_publisher/rviz/bno085.rviz
```

If the `.rviz` file is not available remotely, open RViz2 and manually add:
- **TF** display → Fixed Frame: `base_link`
- **IMU** display (requires `rviz_imu_plugin`) → Topic: `/imu/data`

---

## Published Topics

| Topic       | Type                          | Description                                   |
|-------------|-------------------------------|-----------------------------------------------|
| `/imu/data` | `sensor_msgs/Imu`             | Quaternion orientation, angular vel, linear accel |
| `/imu/mag`  | `sensor_msgs/MagneticField`   | Magnetometer (Tesla)                          |
| `/imu/temp` | `sensor_msgs/Temperature`     | Board temperature (°C)                        |

TF broadcast: `base_link` → `imu_link`

---

## Parameters (config/bno085_params.yaml)

| Parameter        | Default      | Description                              |
|------------------|--------------|------------------------------------------|
| `frame_id`       | `imu_link`   | TF frame of the sensor                   |
| `parent_frame_id`| `base_link`  | Parent TF frame                          |
| `publish_rate_hz`| `50.0`       | Publishing rate (max ~400 Hz)            |
| `i2c_address`    | `0x4A`       | `0x4A` or `0x4B` depending on PS1 pin   |
| `publish_tf`     | `true`       | Broadcast TF transform                   |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Remote I/O Error` on I2C | Check wiring; verify with `i2cdetect -y 1` |
| Topics not visible on remote machine | Confirm same `ROS_DOMAIN_ID`; check firewall (`sudo ufw allow 7400:7500/udp`) |
| IMU display missing in RViz2 | Install `ros-$ROS_DISTRO-rviz-imu-plugin` |
| `No module named 'adafruit_bno08x'` | `pip3 install adafruit-circuitpython-bno08x --break-system-packages` |
| Permission denied on I2C | `sudo usermod -aG i2c $USER` then log out/in |
