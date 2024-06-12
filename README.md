# Drivers_sonar_tritech #

This project is a ROS 2 wrapper built on top of [drivers_sonar_tritec](https://github.com/rock-drivers/drivers-sonar_tritech/tree/master)

## Use case
1. run in the blue-os terminal: `socat -d -d /dev/ttyUSB0,raw,echo=0,b=115200 tcp4-listen:5555,fork,reuseaddr` (it can also be `socat -d -d /dev/ttyUSB0,raw,echo=0,b115200 tcp4-listen:5555,fork,reuseaddr`)
2. in local docker ROS 2 - T1:
```bash
sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyUSB0 tcp:192.168.2.2:5555,nodelay,forever
```
T2:
```bash
sudo chmod 666 /dev/ttyUSB0
```


3. launch the sonar node
```bash
ros2 launch drivers_sonar_tritech micron_sonar_node.launch.py
```

4. If you want to visualize the result in RViz, in a separate terminal, run:

```bash
ros2 launch drivers_sonar_tritech rviz_test_micron_sonar_node.launch.py
```


In `config/micron_sonar_node_params.yaml` you can set some of your parameters.

If you want to route, run `export_port.sh` on the device that has the Sonar attached (such as the ROV) and `import_port.sh` on the machine that will run the code (such as your laptop).

You may need to `chmod 666 /dev/ttyUSB0`.
