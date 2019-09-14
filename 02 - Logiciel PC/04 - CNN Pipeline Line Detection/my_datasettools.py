import cv2
import my_constants as consts

## FUNCTIONS ###################################################################

def load_and_preprocess_picture(filename):
    # read picture
    frame = cv2.imread(filename,cv2.IMREAD_COLOR)
    assert(frame.shape == consts.picture_initial_shape)
    # smoothing and gray scale
    frame= cv2.blur(frame,consts.blur_kernel_size)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    assert(frame.shape == (consts.picture_initial_height,consts.picture_initial_width))
    # reshape for conv layers
    frame = frame.reshape(consts.picture_initial_height,consts.picture_initial_width,1)
    # out
    return frame
