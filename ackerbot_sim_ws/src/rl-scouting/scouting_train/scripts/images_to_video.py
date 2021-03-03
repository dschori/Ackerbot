import cv2
import os

nr = 164

folder_name = '../data/train_runs_forward'
#folder_name = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/test_runs_rl/d1_16'
image_folder = '/{}/img_logs'.format(folder_name)
video_name = '/{}/{}_state_video.avi'.format(folder_name, folder_name)

nr = 'd2_16'
image_folder = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/test_runs_rl/{}'.format(nr)
video_name = 'C:/Users/damian.schori/switchdrive/Master-Masterarbeit/30_Videos/test_runs_rl/state_{}.avi'.format(nr, nr)

image_files = sorted(os.listdir(image_folder), key=lambda x: int(x.split('_')[-1].split('.')[0]))
images = [img for img in image_files if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 15, (width,height))
for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


