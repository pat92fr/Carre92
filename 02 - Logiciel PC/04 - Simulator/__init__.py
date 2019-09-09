### https://github.com/jlevy44/UnrealAI/blob/master/CarAI/joshua_work/game/src/simulation.py

import numpy as np
import cv2
 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import *
from panda3d.core import Material
from panda3d.core import Spotlight
from os import mkdir
#from direct.actor.Actor import Actor
#from direct.interval.IntervalGlobal import Sequence
#from panda3d.core import Point3

root_dir = 'c:/tmp'
dataset_dir = 'dataset'
 
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
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

        # create material
        myMaterial = Material()
        myMaterial.setShininess(0.5) #Make this material shiny
        myMaterial.setSpecular(LColor(255,255,0,0))
        #myMaterial.setAmbient((0, 0, 1, 1)) #Make this material blue

        # load circuit model
        self.circuitNodePath = self.loader.loadModel("/c/tmp/media/circuit.bam")
        self.circuitNodePath.reparentTo(self.render)
        self.circuitNodePath.setScale(1.0, 1.0, 1.0)
        self.circuitNodePath.setPos(1.0,-5,0)
        self.circuitNodePath.setHpr(0,90, 270)
        self.circuitNodePath.setMaterial(myMaterial)

        # load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        self.scene.reparentTo(self.render)
        self.scene.setScale(1.25, 1.25, 1.25)
        self.scene.setPos(0,0, -0.1)
 
        # Load the environment model.
        self.car = self.loader.loadModel("models/box")
        self.car.reparentTo(self.render)
        self.car.setScale(1.0, 1.0, 1.0)
        self.car.setPos(0, 0, 0.0)

        # Lights
        render.clearLight()
        
        alight = AmbientLight('ambientLight')
        alight.setColor(Vec4(0.4, 0.4, 0.4, 1))
        alightNP = render.attachNewNode(alight)
        render.setLight(alightNP)

        dlight = DirectionalLight('directionalLight')
        dlight.setDirection(Vec3(1, 1, -1))
        #dlight.setColor(Vec4(1.0, 1.0, 0.8, 1))
        dlight.setColorTemperature(6500)
        dlightNP = render.attachNewNode(dlight)
        render.setLight(dlightNP)

##        s1light = Spotlight('spot1Light')
##        s1light.setColor(Vec4(0.0, 1.0, 0.0, 1.0))
##        lens = PerspectiveLens()
##        s1light.setLens(lens)
##        s1light.setPos(0, 0, 10)
##        s1light.lookAt(self.circuitNodePath)
##        s1lightNP = render.attachNewNode(s1light)
##        render.setLight(s1lightNP)
        
        # camera
        self.camLens.setFov(80)
        self.camLens.setNear(0.01)
        self.camera.setPos(0.0,0.1,0.2)
        self.camera.setHpr(0,0,0)
        self.camera.setHpr(0,-12,0)
        self.camera.reparentTo(self.car)
    
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

#        lines = LineSegs()
#        lines.moveTo(100,0,0)
#        lines.drawTo(100,500,0)
#        lines.setThickness(4)
#        node = lines.create()
#        np = NodePath(node)
#        np.reparentTo(myRender2d)
        
        # controls
        self.quit_button = KeyboardButton.ascii_key('q')
        self.quit = False

        # tasks
        self.taskMgr.add(self.move_task, 'moveTask')
 
    def windDown():
        # De-initialization code goes here!
        self.quit = True
  
    def move_task(self, task):

        # Check if the player is holding keys
        is_down = self.mouseWatcherNode.is_button_down
        if is_down(self.quit_button):
            self.quit  = True
            print('q')
        
        # gamepad inputs
        self.direction = self.gamepad.findAxis(InputDevice.Axis.right_x).value
        self.throttle = self.gamepad.findAxis(InputDevice.Axis.left_x ).value
         
        # center
        self.direction -= 0.38
        self.throttle -= 0.41
        
        print(str(self.direction) + " " + str(self.throttle))
        
        # move
        #y_delta = self.throttle * 1000.0 * task.getDt()
        #w_delta = self.direction * 10000.0 * task.getDt()
        y_delta = self.throttle * 100.0 * task.getDt()
        w_delta = self.direction * 1000.0 * task.getDt()
        self.car.setHpr(self.car, w_delta, 0, 0)
        self.car.set_y(self.car, y_delta)
        
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
    if counter != 0: # first frame buffer is empty, skip it!
        frame = app.get_camera_image()
        frame = cv2.resize(frame[:, :, 0:3], (160, 90),   interpolation = cv2.INTER_AREA)
        filename = dataset_dir + '/render_' + str(counter) + '.jpg'
        ##cv2.imwrite(root_dir + '/' + filename, frame) 
        ##dataset_file.write(filename +';' + str(int(128.0-app.direction*127.0*1.4)) + ';' + str(int(app.throttle*127.0*1.4+128.0)) + '\n') # *1.4 gain
        ##dataset_file.flush()
    counter += 1
dataset_file.close()
print('m:' + str(counter))

    
    
