import cv2
import os

nr = 298

image_folder = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/50_Data/Videos/rl_learning_home1/check_{}/images'.format(nr)
video_name = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/50_Data/Videos/rl_learning_home1/check_{}/vid_{}.avi'.format(nr, nr)

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 30, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


