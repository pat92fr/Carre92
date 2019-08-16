import cv2
from PIL import Image


def show_image():
    # open video stream
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


    return_value, image = camera.read()
    assert(image.shape == (480,640,3))

    img2 = Image.fromarray(image, 'RGB')
    img2.show()

def gstreamer_test_2():


    mode = 640

    if mode == 640:

        width = 640
        height = 480

        cap_send = cv2.VideoCapture('v4l2src ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        cap_send.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap_send.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    
    elif mode == 720:

        width = 1280
        height = 720

        # open video stream
        
        cap_send = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        cap_send.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap_send.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap_send.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap_send.set(cv2.CAP_PROP_FPS, 30)
        return_value, image = cap_send.read()
        print(image.shape)
        assert(image.shape == (720,1280,3))
    
    return_value, image = cap_send.read()
    assert(image.shape == (height,width,3))
    
    #commande Jocelyn
    out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=400 ! queue ! rtph264pay config-inerval=1 ! udpsink host=10.0.10.50 port=5500 sync=false',cv2.CAP_GSTREAMER,0, 20, (width,height), True)


    # gstreamer working command on windows client side
    #.\gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0)

    while True:
        ret,frame = cap_send.read()

        if not ret:
            print('empty frame')
            break

        frame = cv2.flip(frame, 1)
        out_send.write(frame)

        #cv2.imshow('send', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break

    cap_send.release()
    out_send.release()

def gstreamer_test():
    # Cam properties
    fps = 30.
    frame_width = 640
    frame_height = 480
    # Create capture
    cap = cv2.VideoCapture(0)

    # Set camera properties0
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Define the gstreamer sink
    #gst_str = "appsrc ! videoconvert ! shmsink socket-path=/tmp/foo sync=true wait-for-connection=false shm-size=10000000"

    # Check if cap is open
    if cap.isOpened() is not True:
        print ("Cannot open camera. Exiting.")
        quit()

    # Create videowriter as a SHM sink
    # out = cv2.VideoWriter(gst_str, 0, fps, (frame_width, frame_height), True)

    #gst_str_rtp = "appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true threads=4 ! h264parse ! mpegtsmux ! rtpmp2tpay ! udpsink host=192.168.43.57 port=5000"
    #gst_str_rtp = "appsrc ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=10.0.10.50 port=5000"
    #gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay  ! udpsink host=10.0.10.50 port=5000"

    # Equivalent appsrc de la commande Hello World
    gst_str_rtp = "appsrc ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=10.0.10.50 port=5001"



    #client side command : .\gst-launch-1.0 -v udpsrc port=5000 !videoconvert 

    # Create videowriter as a RTP sink
    out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)

    
    # Loop it
    while True:
        # Get the frame
        ret, frame = cap.read()
        # Check
        if ret is True:
            # Flip frame
            frame = cv2.flip(frame, 1)
            # Write to SHM
            out.write(frame)
        else:
            print ("Camera error.")
            time.sleep(1)

    cap.release()
    

if __name__ == '__main__':
    gstreamer_test_2()
    #print(cv2.getBuildInformation())
