import rosbag
import matplotlib.pyplot as plt
import seaborn as sns
import math
import pandas as pd
from tf_bag import BagTfTransformer

sns.set_theme(style="whitegrid")

directions = [1, 2]
approaches = ['teb', 'rl']

end_positions = {
    1: (-4.2, -4.8),
    2: (-1.35, -1.55)
}
df_list = []
for direction in directions:
    for approach in approaches:
        for i in range(15):
            folder = 'd{}_{}'.format(direction, i + 1)
            print('processing: {}'.format(folder))
            try:
                bag = rosbag.Bag('/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))

                bag_transformer = BagTfTransformer(
                    '/home/dschori/Documents/new_rosbags/{}/{}/data_0.bag'.format(approach, folder))
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
            df_list.append(
                {
                    'Approach': approach,
                    'Direction': direction,
                    'Target Position': end_positions[direction],
                    'Actual Position': ending_point,
                    'Distance to Target': math.dist(ending_point, end_positions[direction])
                }
            )
            print(math.dist(ending_point, end_positions[direction]))
            bag.close()

data = pd.DataFrame(df_list)
print(data)

ax = sns.violinplot(data=data, x="Approach", y="Distance to Target", hue='Direction',
               split=True, inner="quart", linewidth=1,
               palette={1: "b", 2: ".85"})
ax.set(ylim=(-3., 3.))
sns.despine(left=True)
plt.show()