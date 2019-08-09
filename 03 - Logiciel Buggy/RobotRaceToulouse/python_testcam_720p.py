import cv2
from PIL import Image


# open video stream
camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
camera.set(cv2.CAP_PROP_FOURCC, fourcc)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
camera.set(cv2.CAP_PROP_FPS, 30)

while True:
	return_value, image = camera.read()
	assert(image.shape == (720,1280,3))
	cv2.imshow("image",image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

camera.release()
cv2.DestroyAllWindows()


