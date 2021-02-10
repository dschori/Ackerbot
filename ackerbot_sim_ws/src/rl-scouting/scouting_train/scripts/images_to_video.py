import cv2
import os

nr = 35

image_folder = 'C:/Users/damian.schori/Downloads/training/check_{}'.format(nr)
video_name = 'C:/Users/damian.schori/Downloads/training/check_{}.avi'.format(nr)

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 30, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


