"""One-off: measure base execution gain from a rosbag of /cmd_vel vs /odom."""
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

TYPEMAP = {
    "geometry_msgs/msg/Twist": Twist,
    "nav_msgs/msg/Odometry": Odometry,
}

reader = SequentialReader()
reader.open(
    StorageOptions(uri="/tmp/probe_bag", storage_id="mcap"),
    ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

cmd, odo = [], []
while reader.has_next():
    topic, data, t = reader.read_next()
    msg = deserialize_message(data, TYPEMAP[types[topic]])
    tsec = t / 1e9
    if topic == "/cmd_vel":
        cmd.append((tsec, msg.linear.x, msg.linear.y))
    else:
        odo.append((tsec, msg.pose.pose.position.x, msg.pose.pose.position.y))

cmd = np.array(cmd)
odo = np.array(odo)
print(f"cmd samples: {len(cmd)}, odom samples: {len(odo)}")

t0 = odo[0, 0]
m_cmd = (cmd[:, 0] - t0 >= 2.0) & (cmd[:, 0] - t0 <= 13.0)
m_odo = (odo[:, 0] - t0 >= 2.0) & (odo[:, 0] - t0 <= 13.0)
cvx, cvy = cmd[m_cmd, 1], cmd[m_cmd, 2]
cplan = np.hypot(cvx, cvy)
t_o = odo[m_odo, 0] - t0
x_o, y_o = odo[m_odo, 1], odo[m_odo, 2]
ovx, ovy = np.gradient(x_o, t_o), np.gradient(y_o, t_o)
oplan = np.hypot(ovx, ovy)
print(f"指令 vx std={cvx.std():.4f} | vy std={cvy.std():.4f} | planar std={cplan.std():.4f} m/s")
print(f"实际 vx std={ovx.std():.4f} | vy std={ovy.std():.4f} | planar std={oplan.std():.4f} m/s")
print(f"X 向执行增益 std比(vx) = {ovx.std() / max(cvx.std(), 1e-9):.2f}")
print(f"平面执行增益 std比     = {oplan.std() / max(cplan.std(), 1e-9):.2f}")
print(f"底盘位移: x [{x_o.min():.3f}, {x_o.max():.3f}] m, y [{y_o.min():.3f}, {y_o.max():.3f}] m")

g = np.arange(2.0, 13.0, 0.05)
ci = np.interp(g, cmd[m_cmd, 0] - t0, cplan)
oi = np.interp(g, t_o, oplan)
c = np.correlate(ci - ci.mean(), oi - oi.mean(), "full")
lag = (c.argmax() - (len(g) - 1)) * 0.05
print(f"平面速度 实际相对指令滞后 ≈ {lag:+.2f} s (正=实际滞后)")
