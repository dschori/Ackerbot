import rosbag
import matplotlib.pyplot as plt
from tf_bag import BagTfTransformer

folder = 'd1_5'

bag = rosbag.Bag('/home/dschori/Documents/new_rosbags/rl/{}/data_0.bag'.format(folder))

bag_transformer = BagTfTransformer('/home/dschori/Documents/new_rosbags/rl/{}/data_0.bag'.format(folder))

x_s, y_s = [], []

for topic, msg, t in bag.read_messages(topics=['/tf']):
    try:
        translation, quaternion = bag_transformer.lookupTransform('map', 'base_link', t)
        x_s.append(translation[0])
        y_s.append(translation[1])
    except:
        pass
bag.close()

plt.plot(x_s, y_s)
plt.show()
print(len(x_s))
