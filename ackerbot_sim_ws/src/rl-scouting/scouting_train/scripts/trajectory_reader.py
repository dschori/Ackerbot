import rosbag
import matplotlib.pyplot as plt
from tf_bag import BagTfTransformer

for i in range(6):
    bag = rosbag.Bag('/home/dschori/runs/baseline/{}/data_0.bag'.format(i+1))

    bag_transformer = BagTfTransformer('/home/dschori/runs/baseline/{}/data_0.bag'.format(i+1))

    x_s, y_s = [], []

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        try:
            translation, quaternion = bag_transformer.lookupTransform('odom', 'base_link', t)
            x_s.append(translation[0])
            y_s.append(translation[1])
        except:
            pass
    bag.close()

    plt.plot(x_s, y_s)
plt.show()
print(len(x_s))
