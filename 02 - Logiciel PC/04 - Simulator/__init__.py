import numpy as np
import cv2
 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import *
from os import mkdir
#from direct.actor.Actor import Actor
#from direct.interval.IntervalGlobal import Sequence
#from panda3d.core import Point3

root_dir = 'c:/tmp'
dataset_dir = 'dataset'
 
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        self.quit = False
        
        # Window
        winprops  = WindowProperties()
        winprops .setSize(640, 360)
        base.win.requestProperties(winprops ) 
        
        base.setFrameRateMeter(True)
        
        # gamepad
        self.gamepad = None
        devices = self.devices.getDevices(InputDevice.DeviceClass.gamepad)
        if devices:
            print("Devices founds..." + str(len(devices)))
            # gamepad yet.
            if devices[0].device_class == InputDevice.DeviceClass.gamepad and not self.gamepad:
                print("Found %s" % (devices[0]))
                self.gamepad = devices[0]
                
        # Disable the camera trackball controls.
        self.disableMouse()
 
        # load circuit model
        self.circuitNodePath = self.loader.loadModel("/c/tmp/circuit.bam")
        self.circuitNodePath.reparentTo(self.render)
        self.circuitNodePath.setScale(1.0, 1.0, 1.0)
        self.circuitNodePath.setPos(1.5,-5,0)
        self.circuitNodePath.setHpr(0,90, 270)

        # load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        self.scene.reparentTo(self.render)
        self.scene.setScale(1.25, 1.25, 1.25)
        self.scene.setPos(0,0, -1)
 
#        # Load the environment model.
#        self.box = self.loader.loadModel("models/box")
#        # Reparent the model to render.
#        self.box.reparentTo(self.render)
#        # Apply scale and position transforms on the model.
#        self.box.setScale(1.0, 1.0, 1.0)
#        self.box.setPos(0, 0, 0.0)

        self.forward_speed = 150.0 # units per second
        self.backward_speed = 10.0
        self.rotate_speed = 6000.0
        self.forward_button = KeyboardButton.ascii_key('z')
        self.backward_button = KeyboardButton.ascii_key('s')
        self.right_button = KeyboardButton.ascii_key('d')
        self.left_button = KeyboardButton.ascii_key('q')
        self.quit_button = KeyboardButton.ascii_key('x')

        #♠self.camera.lens.setAspectRatio(1280.0 / 720.0)
        self.camLens.setFov(70)
        self.camLens.setNear(0.001)
        self.camera.setPos(0.5,0,0.2)
        self.camera.setHpr(0,0,0)
        #self.camera.setHpr(0,-20,0)
        #self.camera.reparentTo(self.box)

        # osd
        self.dr = self.win.makeDisplayRegion()
        self.dr.setSort(20)
         
        myCamera2d = NodePath(Camera('myCam2d'))
        lens = OrthographicLens()
        lens.setFilmSize(2, 2)
        lens.setNearFar(-1000, 1000)
        myCamera2d.node().setLens(lens)
         
        myRender2d = NodePath('myRender2d')
        myRender2d.setDepthTest(False)
        myRender2d.setDepthWrite(False)
        myCamera2d.reparentTo(myRender2d)
        self.dr.setCamera(myCamera2d)

        lines = LineSegs()
        lines.moveTo(100,0,0)
        lines.drawTo(100,500,0)
        lines.setThickness(4)
        node = lines.create()
        np = NodePath(node)
        np.reparentTo(myRender2d)
        

        self.taskMgr.add(self.move_task, 'modeTask')
 
    def windDown():
        # De-initialization code goes here!
        self.quit = True
  
    def move_task(self, task):
        
        # gamepad inputs
        self.direction = self.gamepad.findAxis(InputDevice.Axis.right_x).value
        self.throttle = self.gamepad.findAxis(InputDevice.Axis.left_x ).value
         
        # center
        self.direction -= 0.38
        self.throttle -= 0.41
        
        print(str(self.direction) + " " + str(self.throttle))

        xspeed = self.throttle * 1000.0
        wspeed = self.direction * 10000.0
     
        # Check if the player is holding W or S
        is_down = self.mouseWatcherNode.is_button_down
     
        if is_down(self.quit_button):
            self.quit  = True
            print('q')

        if is_down(self.forward_button):
            xspeed += self.forward_speed
            print('f')
     
        if is_down(self.backward_button):
            xspeed -= self.backward_speed
            print('b')
     
        if is_down(self.right_button):
            wspeed -= self.rotate_speed
            print('r')
     
        if is_down(self.left_button):
            wspeed += self.rotate_speed
            print('l')
            
        # Move the player
        y_delta = xspeed * task.getDt()
        w_delta = wspeed * task.getDt()
        #self.box.setHpr(self.box, w_delta, 0, 0)
        #self.box.set_y(self.box, y_delta)
        self.camera.setHpr(self.camera, w_delta, 0, 0)
        self.camera.set_y(self.camera, y_delta)
        
        return Task.cont

    def get_camera_image(self, requested_format=None):
        """
        Returns the camera's image, which is of type uint8 and has values
        between 0 and 255.
        The 'requested_format' argument should specify in which order the
        components of the image must be. For example, valid format strings are
        "RGBA" and "BGRA". By default, Panda's internal format "BGRA" is used,
        in which case no data is copied over.
        """
        tex = self.dr.getScreenshot()
        if requested_format is None:
            data = tex.getRamImage()
        else:
            data = tex.getRamImageAs(requested_format)
        image = np.frombuffer(data, np.uint8)  # use data.get_data() instead of data in python 2
        image.shape = (tex.getYSize(), tex.getXSize(), tex.getNumComponents())
        image = np.flipud(image)
        return image
        
    # Define a procedure to move the camera.
#    def spinCameraTask(self, task):
#        angleDegrees = task.time * 6.0
#        angleRadians = angleDegrees * (pi / 180.0)
#        distance = 10
#        self.camera.setPos(distance * sin(angleRadians), -distance * cos(angleRadians), 2)
#        self.camera.setHpr(angleDegrees, 0, 0)
#        return Task.cont
 
app = MyApp()
#app.run()
try:
    mkdir(root_dir+'/'+dataset_dir)
except FileExistsError:
    pass
dataset_file = open(root_dir+'/'+dataset_dir+'/'+'dataset.txt',  'w')
counter = 0
while not app.quit:
    taskMgr.step()
    frame = app.get_camera_image()
    frame = cv2.resize(frame[:, :, 0:3], (160, 90),   interpolation = cv2.INTER_AREA)
    filename = dataset_dir + '/render_' + str(counter) + '.jpg'
    cv2.imwrite(root_dir + '/' + filename, frame) 
    dataset_file.write(filename +';' + str(int(128.0-app.direction*127.0)) + ';' + str(int(app.throttle*127.0+128.0)) + '\n')
    dataset_file.flush()
    counter += 1
dataset_file.close()
print('m:' + str(counter))

    
    
