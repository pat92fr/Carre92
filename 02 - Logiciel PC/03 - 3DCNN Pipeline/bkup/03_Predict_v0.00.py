#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

#from tkinter  import *
#from PIL import Image
#from PIL import ImageTk

import sys
import os
from argparse import ArgumentParser

## CONSTANTS ###################################################################

# picture
picture_initial_width = 160 
picture_initial_height = 90 
picture_initial_shape = (picture_initial_height, picture_initial_width, 3)

# sequence buffer
depth = 10
sequence_shape = (depth,picture_initial_height,picture_initial_width,1)
x = np.zeros( sequence_shape )

## PARAMETERS ##################################################################

# inputs
input_directory = 'testset'
input_filename = 'capture_Thu_Aug_15_19-50-46_2019'

# outputs
ouput_directory = 'predict'
input_filename = 'out' 


## MAIN ########################################################################

print("Opening file opened...")
capture = cv2.VideoCapture(input_directory+'/'+input_filename+'.avi')
if not capture:
    print("Video file not opened!")
    exit()
else:
    print("Done.")
while(capture.isOpened()):
    # Capture frame-by-frame
    returnval, image = capture.read()
    if not returnval:
        print("Video file done!")
        break    
    assert(image.shape == picture_initial_shape)
    # smoothing and gray scale
    image_smoothed= cv2.blur(image,(3,3))
    image_bw = cv2.cvtColor(image_smoothed, cv2.COLOR_RGB2GRAY)
    assert(image_bw.shape == (picture_initial_height,picture_initial_width))
    frame = image_bw.reshape(picture_initial_height,picture_initial_width,1)

    # FIFO
    x[:-1] = x[1:]; x[-1] = frame
    assert(x.shape == sequence_shape)

    
    if True:
        plt.clf()
        plt.imshow(x[-1].reshape(picture_initial_height,picture_initial_width), cmap = 'gray' )
        #plt.axvline(x=(y[0]+1.0)/2.0*picture_initial_width,linewidth=2, label='DIR')
        plt.axhline(y=picture_height_crop,linewidth=2, label='CROP', marker="v", linestyle='--')
        plt.axhline(y=picture_initial_height/2,linewidth=2)
        plt.pause(0.001)  

# When everything done, release the capture
capture.release()
cv2.destroyAllWindows()
print("Exit.")

##
##width = 160
##height = 90
##height_cropped = 64
##height_crop = height-height_cropped
##resize_size = (width, height)
##
##
##
##global window
##global dataset
##global current_frame_filename
##global last_position
### Function to be called when mouse 1 is clicked
##def printcoords(event):
##    global window
##    global dataset
##    global current_frame_filename
##    global last_position
##    x = window.winfo_pointerx()-5
##    y1 = float(x) / 1280.0 * 2.0 - 1.0
##    if y1<-1.0:
##        y1=-1.0
##    elif y1>1.0:
##        y1=1.0
##    y2 = 1.0
##    print('filename:' + current_frame_filename + "   y1:" + str(y1) + "   y2:" + str(y2))
##    dataset.write("%s;%f;%f\r\n" % (current_frame_filename, y1, y2))
##    window.destroy()
##    last_position = y1
##
##
##print("Processing video file...")
##dataset = open(ouput_directory + '/' + 'dataset.txt','w+')
##frame_counter = 0
##last_position = 0.0
##while True:
##    # read frame
##    returnval, frame = capture.read()
##    if not returnval:
##        print("Video file done!")
##        break
##    # skip 9 frames over 10
##    if frame_counter % 10 == 0:
##        # resize frame
##        frame_resized = cv2.resize(frame,resize_size)
##        assert frame_resized.shape == (height,width, 3)
##        # crop frame
##        frame_cropped = frame_resized[height_crop:,:]
##        assert frame_cropped.shape == (height_cropped,width, 3)
##        # display frame 
##        ###cv2.imshow('Viewer',frame_cropped)
##        ###cv2.waitKey(1)
##        # save frame
##        current_frame_filename = 'frame_' + str(frame_counter) + '_' + filename + '.jpg'
##        cv2.imwrite(ouput_directory + '/' + current_frame_filename,frame_cropped)
##        # GUI
##        window = Tk()
##        window.title('Viewer')  
##        window.geometry("1280x512+0+0")
##        canvas = Canvas(window, width = 1280, height = 512)      
##        canvas.pack(side='top', fill='both', expand='yes')      
##        frame_color = cv2.cvtColor(cv2.resize(frame_cropped,(1280,512)), cv2.COLOR_BGR2RGB)
##        picture = ImageTk.PhotoImage(Image.fromarray(frame_color))
##        canvas.create_image(0,20,image=picture,anchor="nw")
##        canvas.create_rectangle(0, 20+80-5, 1280, 20+80+5, outline="blue")
##        canvas.create_rectangle(int(1280.0*(last_position+1.0)/2.0)-10, 20+80-10,int(1280.0*(last_position+1.0)/2.0)+10, 20+80+10, outline="blue")
##        ##last_position
##        canvas.bind("<ButtonPress-1>",printcoords)
##        window.mainloop()
##        # flush
##        dataset.flush()
##    # next frame...
##    frame_counter += 1
##
##dataset.close()
### When everything done, release the capture
##capture.release()
##cv2.destroyAllWindows()
##print("Exit.")
##



#
#event2canvas = lambda e, c: (c.canvasx(e.x), c.canvasy(e.y))
#
## Check parameters
#parser = ArgumentParser()
#parser.add_argument("-g", "--gif", dest="gif_directory",
#                    help="file name for file mode", type=str, default=None)
#args = parser.parse_args()
#
#if args.gif_directory == None:
#    print('#ERROR: missing gif directory')
#    sys.exit(0)
#
## Convert jpeg to gif, if any
#image_dir = args.gif_directory
#included_extensions = ['jpg', 'jpeg']
#file_names = [fn for fn in os.listdir(image_dir) if any(fn.endswith(ext) for ext in included_extensions)]
#for j in file_names:
#    print(f'# Converting {image_dir}/{j} to {image_dir}/{j}.gif')
#    img     = PIL.Image.open(image_dir + '/' + j)
#    img_rgb = img.convert('RGB')
#    try:
#        img_rgb.save(image_dir + '/' + j + '.gif', 'GIF')
#    except:
#        print('error')
#
## Find how many images should be displayed
#image_dir = args.gif_directory
#included_extensions = ['gif']
#file_names = [fn for fn in os.listdir(image_dir) if any(fn.endswith(ext) for ext in included_extensions)]
#
#if len(file_names)==0:
#    print(f'#ERROR: no image file in directory')
#    sys.exit(0)
#
#exists = os.path.isfile('result.txt')
#if exists:
#    print(f'#ERROR: result file already exist. Rename or remove it.')
#    sys.exit(0)
#
## Create file
#global result_file
#result_file = open("result.txt","w+")
#
## Scan all the files and display the image
#for i in file_names:
#    
#    root = Tk()
#    root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
#    
#    # Setting up a tkinter canvas with scrollbars
#    frame = Frame(root, bd=2, relief=SUNKEN)
#    frame.grid_rowconfigure(0, weight=1)
#    frame.grid_columnconfigure(0, weight=1)
#    xscroll = Scrollbar(frame, orient=HORIZONTAL)
#    xscroll.grid(row=1, column=0, sticky=E+W)
#    yscroll = Scrollbar(frame)
#    yscroll.grid(row=0, column=1, sticky=N+S)
#    canvas = Canvas(frame, bd=0, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
#    canvas.grid(row=0, column=0, sticky=N+S+E+W)
#    xscroll.config(command=canvas.xview)
#    yscroll.config(command=canvas.yview)
#    frame.pack(fill=BOTH,expand=1)
#    
#    f = image_dir + '/' + i
#    root.title(f)
#    print(f'# Processing file {f}')
#    img = PhotoImage(file=f)
#    canvas.create_image(0,0,image=img,anchor="nw")
#    canvas.config(scrollregion=canvas.bbox(ALL))
#    canvas.create_rectangle(0, 480, 1280, 600, outline="blue")
#
#    # Function to be called when mouse 1 is clicked
#    def printcoords(event):
#        global result_file
#        global f
#        # Outputting x and y coords to console
#        cx, cy = event2canvas(event, canvas)
#        result_file.write("%s;%d;%d;" % (f, cx, cy))
#        root.destroy()
#
#    # Function to be called when mouse 2 is clicked
#    def printemptycoords(event):
#        global result_file
#        global f
#        # Outputting x and y coords to console
#        cx, cy = event2canvas(event, canvas)
#        result_file.write("%s;%d;%d;" % (f, -1, -1))
#        root.destroy()
#
#    # Mouseclick event
#    canvas.bind("<ButtonPress-1>",printcoords)
#    canvas.bind("<ButtonPress-2>",printemptycoords)
#    root.mainloop()
#
#print(f'# {len(file_names)} files in file result.txt')
#
#result_file.close()
#
## End of file

