"""DDS spike: does Isaac's bundled jazzy rclpy (python 3.11) talk to the container's
real jazzy (python 3.12)?

This is the gate for the whole IsaacHardwareInterface design. Isaac ships its own rclpy
built for its own python 3.11; the ros2_control side runs on jazzy's python 3.12. They can
only ever meet on the wire. If this spike fails, the topic transport is dead and the design
needs rethinking.

Run inside pupper-jazzy, with the bundled ROS2 env exported (see run_spike.sh):
    ./python.sh spike_isaac_pub.py
Then, from a normal jazzy shell in the same container:
    ros2 topic echo /isaac/spike std_msgs/msg/Float64MultiArray
"""

import sys

sys.argv.append("--/rtx/verifyDriverVersion/enabled=false")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = rclpy.create_node("isaac_spike_pub")
pub = node.create_publisher(Float64MultiArray, "/isaac/spike", 10)
sub_got = []

# Also subscribe, to prove the reverse direction (container -> Isaac), which is the
# direction the joint_command path actually needs.
node.create_subscription(
    Float64MultiArray, "/isaac/spike_back", lambda m: sub_got.append(list(m.data)), 10
)

print(f"[spike] rclpy={rclpy.__file__}", flush=True)
print(f"[spike] rmw={rclpy.get_rmw_implementation_identifier()}", flush=True)

msg = Float64MultiArray()
for i in range(300):  # ~30 s at 10 Hz
    msg.data = [float(i), 1.5, 2.5]
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.0)
    if sub_got:
        with open("/tmp/spike_reverse.txt", "w") as f:
            f.write(f"REVERSE_OK received from container: {sub_got[-1]}")
        print(f"[spike] REVERSE_OK {sub_got[-1]}", flush=True)
        break
    simulation_app.update()
    import time

    time.sleep(0.1)

rclpy.shutdown()
simulation_app.close()
