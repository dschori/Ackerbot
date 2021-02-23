import rosbag
import matplotlib.pyplot as plt
import seaborn as sns
from ackermann_msgs.msg import AckermannDriveStamped

sns.set_theme(style="whitegrid")

bag = rosbag.Bag('/home/dschori/Documents/new_rosbags/drive_controller/data_0.bag')

fig, ax = plt.subplots(1, 1, figsize=(8, 5))

ax.set_xlabel('Time (s)', fontsize=16)
ax.set_ylabel('Speed (m/s)', fontsize=16)

ax.set_xlim([0., 17.5])
ax.set_ylim([-0.05, 0.4])
ax.set_title('Speed controller', fontsize=24)

times = []
times2 = []
speed_cmds = []
speed_measurements = []
time_offset = -1613134888.3346577 - 22.
last_cmd = 0.
for topic, msg, t in bag.read_messages(topics=['/camera_t265/odom/sample', '/ackermann_cmd']):
    if hasattr(msg, 'twist'):
        speed_measurements.append(msg.twist.twist.linear.x)
        times.append(t.to_sec() + time_offset)
    if hasattr(msg, 'drive'):
        speed_cmds.append(msg.drive.speed)
        times2.append(t.to_sec() + time_offset)
        last_cmd = msg.drive.speed
    else:
        speed_cmds.append((last_cmd))
        times2.append(t.to_sec() + time_offset)



line, = ax.plot(times, speed_measurements)
line.set_label('Speed measurement')
line, = ax.plot(times2, speed_cmds)
line.set_label('Speed command')
ax.legend(bbox_to_anchor=(0.63,0.2))
plt.savefig('speed_controller.png', dpi=300)
plt.show()
bag.close()