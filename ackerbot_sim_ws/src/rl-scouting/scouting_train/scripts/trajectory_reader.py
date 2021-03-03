import rosbag
import matplotlib.pyplot as plt
import seaborn as sns
from tf_bag import BagTfTransformer

sns.set_theme(style="whitegrid")

direction = 1
approach = 'teb'

fig, ax = plt.subplots(1, 1, figsize=(8, 8))

ax.set_xlabel('X- Coordinates (m)', fontsize=16)
ax.set_ylabel('Y- Coordinates (m)', fontsize=16)

ax.set_xlim([-7., -0.5])
ax.set_ylim([.5, -6.])
ax.set_title('Trajectories {} DIR{}'.format(approach.upper(), direction), fontsize=24)

obstacle = plt.Rectangle((-4.66, -3.32), width=100, height=0.32, alpha=0.5, color='black')
ax.add_patch(obstacle)
obstacle = plt.Rectangle((-2.64, -3.0), width=100, height=0.91, alpha=0.5, color='black')
ax.add_patch(obstacle)
obstacle = plt.Rectangle((-2.0, -1.0), width=1, height=100, alpha=0.5, color='black')
ax.add_patch(obstacle)
obstacle = plt.Rectangle((-6.4, -4.3), width=0.35, height=0.8, alpha=0.5, color='black')
ax.add_patch(obstacle)
obstacle = plt.Rectangle((-7, -6), width=0.6, height=100, alpha=0.5, color='black')
ax.add_patch(obstacle)

if direction == 1:
    ending_tolerance = plt.Circle((-4.2, -4.8), 0.5, alpha=0.2, color='black')
    ax.add_patch(ending_tolerance)
    ax.annotate('Target area', (-4.2-0.4, -4.8-0.65))
elif direction == 2:
    starting_tolerance = plt.Circle((-1.35, -1.55), 0.5, alpha=0.2, color='black')
    ax.add_patch(starting_tolerance)
    ax.annotate('Target area', (-1.35-0.4, -1.55+0.65))


def get_cmap(n, name='tab20'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)


cmap = get_cmap(150)
legend_handles = []
target_distribution = []
for i in range(15):
    folder = 'd{}_{}'.format(direction, i + 1)
    print('processing: {}'.format(folder))
    try:
        bag = rosbag.Bag('/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))

        bag_transformer = BagTfTransformer('/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))
    except rosbag.bag.ROSBagUnindexedException:
        continue

    x_s, y_s = [], []

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        try:
            translation, quaternion = bag_transformer.lookupTransform('map', 'base_link', t)
            x_s.append(translation[0])
            y_s.append(translation[1])
        except:
            pass
    starting_point = (x_s[0], y_s[0])
    ending_point = (x_s[-1], y_s[-1])
    target_distribution.append(ending_point)
    ax.plot(*starting_point, marker='o', markersize=12, c=cmap(i * 10))
    ax.plot(*ending_point, marker='D', markersize=12, c=cmap(i * 10))
    line, = ax.plot(x_s, y_s, linestyle='dashed', linewidth=2, c=cmap(i * 10))
    line.set_label('Run {}'.format(i + 1))
    legend_handles.append(line)
    #ax.legend()
    bag.close()

starting_obj, = ax.plot((-100, -100), marker='o', markersize=12, c='black')
starting_obj.set_label('Start')
ending_obj, = ax.plot((-100, -100), marker='D', markersize=12, c='black')
ending_obj.set_label('End')


ax.legend()
plt.gca().invert_yaxis()
plt.savefig('/home/dschori/Documents/new_rosbags/outputs/img_{}_dir{}.png'.format(approach, direction), dpi=300)
plt.show()
