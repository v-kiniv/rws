# ROS WebSocket Server
RWS provides WebSocket interface to ROS2. It is partially* compatible with Rosbridge v2 protocol and can be used as a drop-in replacement. The main difference with Rosbridge is that RWS is written is C++, hence it's more fast and memory efficient, however ROS1 is not supported.

_* Not all Rosbridge operations supported, check compatability section below for more information_.

For general information regarding the protocol, check [Rosbridge protocol description](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md).

# Usage
Clone RWS into your workspace source directory:
```bash
cd your_workspace_dir/src
git clone https://github.com/v-kiniv/rws.git
# Build your workspace using colcon, as usual
```

Launch using launch script provided by the package:
```bash
# source your workspace
ros2 launch rws rws_server_launch.py
```

Or add `rws_server` node to your own launch script:
```bash
rws_server_node = Node(
	package='rws',
	executable='rws_server',
	name='rws_server',
	output='screen',
	parameters=[{
	  # ...
	}]
)
```

# Parameters
* `port` - port to use for websocket server. `9090` by default.
* `rosbridge_compatible` - enable compatibility with rosbridge protocol. Currently the only difference is that **stamp->nanosec** field in all messages will be renamed to **stamp->nsec**. `True` by default.
* `watchdog` - use ping/pong to detect and drop unresponsive clients that keep TCP connection open. `True` by default.

# Rosbridge compatability

### Operations
| Operation | RWS | Rosbridge | Description|
| --- | --- | --- | --- |
| advertise | + | + | Advertise topic |
| unadvertise | + | + | Stop advertising topic |
| publish | + | + | Publish to topic |
| subscribe | + | + | Subscribe to topic |
| unsubscribe | + | + | Unsubscribe from topic |
| call_service | + | + | Call service |
| advertise_service | - | + |Advertise external service |
| unadvertise_service | - | + | Stop advertising external service |
| service_request | - | + | Request to external service |
| service_response | - | + | Response from external service |

### Fragmentation
**Message fragmentation is not supported in RWS**

### Compression
| Algorithm | RWS | Rosbridge |
| --- | --- | --- |
| CBOR-RAW | + | + |
| CBOR | + | + |
| BSON | + | + |
| MessagePack | + | - |
| UBJSON | + | - |
| BJData | + | - |

### Rosapi service calls
Unlike Rosbridge, RWS does not expose `/rosapi` node, all `rosapi` related API requests are handled internally in `rws_server` node.

Check the list of implemented `rosapi` services in https://github.com/v-kiniv/rws/issues/49#issue-2839982035

# Dependencies
RWS is using [websocketpp](https://github.com/zaphoyd/websocketpp), [asio](https://github.com/chriskohlhoff/asio) and [nlohmann/json](https://github.com/nlohmann/json). These 3 libraries will be fetched by CMake during configuration step.

# Motivation behind RWS
In my experiece, Rosbridge does not work well with the second version of ROS. Seems like Rosbridge developers mainly focused on ROS1 support, and ROS2 is handled on a best effort basis(for example, check [this issue](https://github.com/RobotWebTools/rosbridge_suite/issues/744)). I'm experiencing severe memory leaks(~1 GB of memory per day), high CPU usage, on a device like Raspberry Pi it's a problem. It's also not very stable and sometimes crashes if multiple clients reconnect rapidly.
My first intention was to fix ROS2 support in Rosbridge. I started by collecting all non-merged fixes from other people and trying to add my own, some progress was made, but the server was still CPU hungry(even though less leaky and more stable), so I decided that C++ is more appropriate than Python, for this kind of task.

### Performance
Here is the data from `htop` command output from a Raspberry Pi 4B(4GB version) running the following stack:
* ROS2 Galactic
* Rosbridge(as of [this commit](https://github.com/RobotWebTools/rosbridge_suite/commit/7af3683ec6a02b1569d7e3c42367721b0be2ac78)) node
* iRobot Create 2 nodes
* Lidar driver node(RPLidar A1)
* IMU driver node(BNO055)

Client is a Mac running Ventura and Foxglove Studio(v1.32.0) subscibed to 7x20Hz and 3x1Hz topics(plots with IMU data, odometry, etc.).

| Process | CPU Usage | Memory Usage(initial) |
| --- | --- | --- |
| rosbridge_server | 57.4% | 59 MB |
| rws_server | 12.8% | 18 MB |

This is **not** a benchmark by any means, and is intended to demostrate that there **is** a difference in performance with my setup. I encourage you to try RWS with your setup and draw conslusion based on that.
