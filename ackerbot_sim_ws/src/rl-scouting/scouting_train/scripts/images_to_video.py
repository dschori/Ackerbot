import cv2
import os

nr = 164

# image_folder = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/rl_training_process2/img_logs/{}'.format(nr)
# video_name = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/rl_training_process2/img_logs/check_{}.avi'.format(nr)
folder_name = 'train_runs_forward'
image_folder = '../data/{}/img_logs'.format(folder_name)
video_name = '../data/{}/{}_state_video.avi'.format(folder_name, folder_name)

image_files = sorted(os.listdir(image_folder), key=lambda x: int(x.split('_')[-1].split('.')[0]))
images = [img for img in image_files if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 15, (width,height))
for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


