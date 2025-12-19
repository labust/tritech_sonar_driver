# Drivers_sonar_tritech (ROS2)

This project is a **ROS 2 wrapper for the Tritech Micron sonar**, built on top of [drivers-sonar-tritech](https://github.com/rock-drivers/drivers-sonar_tritech/tree/master).


## Prerequisites

- Tritech Micron sonar connected to the ROV
- BlueOS running on the ROV
- ROS2 (tested with Humble) on the local machine
- `socat` installed on both machines
- Network connectivity between the ROV and the local machine


## Usage

### 1. Connect to the BlueOS terminal (ROV)

From your local machine:

```bash
ssh pi@192.168.2.2
```

Verify that the sonar is connected:

```bash
ls /dev/ttyUSB*
```

---

### 2. Expose the sonar serial port over TCP (ROV)

Run in the **BlueOS terminal**:

```bash
socat -d -d /dev/ttyUSB0,raw,echo=0,b115200 tcp4-listen:5555,fork,reuseaddr
```

---

### 3. Create a local virtual serial port (local machine)

On the **local ROS 2 machine**:

```bash
sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyUSB0 tcp:192.168.2.2:5555,nodelay,forever
```

In the new terminal, fix permissions:

```bash
sudo chmod 666 /dev/ttyUSB0
```

---

### 4. Launch the sonar driver

```bash
ros2 launch drivers_sonar_tritech micron_sonar_node.launch.py
```

---

### 5. Visualize sonar data (optional)

```bash
ros2 launch drivers_sonar_tritech rviz_test_micron_sonar_node.launch.py
```


## Configuration

Edit sonar parameters in:

```
config/micron_sonar_node_params.yaml
```


## Sonar not responding

- Is the sonar physically connected to the ROV?
- Check serial port (i.e., `/dev/ttyUSB0`)
- Check baud rate (`115200`)
- Ensure only one process accesses the serial port (i.e., `/dev/ttyUSB0`)
- Ensure `socat` is running on both machines
- Ensure TCP port (i.e., `5555`) is reachable 
