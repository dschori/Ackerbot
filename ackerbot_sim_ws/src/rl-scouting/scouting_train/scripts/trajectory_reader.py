import rosbag
import matplotlib.pyplot as plt
import seaborn as sns
from tf_bag import BagTfTransformer

sns.set_theme(style="whitegrid")

direction = 1
approach = 'teb'

fig, ax = plt.subplots(1, 1, figsize=(10, 10))

ax.set_xlabel('X- Coordinates (m)')
ax.set_ylabel('Y- Coordinates (m)')

ax.set_xlim([-6., -1.])
ax.set_ylim([-1., -5.])

def get_cmap(n, name='tab20b'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

cmap = get_cmap(150)

for i in range(3):
    folder = 'd{}_{}'.format(direction, i+1)
    print('processing: {}'.format(folder))
    bag = rosbag.Bag('/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))

    bag_transformer = BagTfTransformer('/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))

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
    ax.plot(*starting_point, marker='o', markersize=12, c=cmap(i*5))
    ax.plot(*ending_point, marker='D', markersize=12, c=cmap(i*5))
    line, = ax.plot(x_s, y_s, linestyle='dashed', linewidth=2, c=cmap(i*5))
    line.set_label('Run {}'.format(i+1))
    ax.legend()
    bag.close()

plt.show()
