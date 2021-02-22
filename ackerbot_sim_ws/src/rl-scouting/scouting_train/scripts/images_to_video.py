import cv2
import os

nr = 199

image_folder = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/rl_training_process/img_logs/{}'.format(nr)
video_name = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/rl_training_process/img_logs/check_{}.avi'.format(nr)

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 15, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


