import cv2
from PIL import Image

# open video stream
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while(True):

	return_value, image = camera.read()
	assert(image.shape == (480,640,3))

	img2 = Image.fromarray(image, 'RGB')
	img2.show()