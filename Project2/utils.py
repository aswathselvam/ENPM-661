import os
import cv2
import shutil
from glob import glob
from tqdm import tqdm
import imageio



def createFolder(path):
    if not os.path.exists(path):
        os.makedirs(path)

def deleteFolder(path):
    if os.path.exists(path):
        shutil.rmtree(path)


def remove_file(file):
    try:
        os.remove(file)
    except OSError as e:
        print("Error: %s : %s" % (file, e.strerror))

def createMovie(path):
    image_folder = path
    video_name = 'simulation_video.mp4'

    images = sorted(glob("results/*.png"))
    frame = cv2.imread(os.path.join(images[0]))
    height, width, channels = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_name, fourcc, 30, (width,height))
    imgs = []
    for i in tqdm(range(len(images))):
        imgs.append(imageio.imread(images[i]))
        image = cv2.imread(images[i])
        if i==0:
            h, w, _ = image.shape
        video.write(image)

    for i in range(20):
        imgs.append(imageio.imread(images[len(images)-1]))
    imageio.mimsave('simulation_video.gif', imgs, fps=30)
    cv2.destroyAllWindows()
    video.release()