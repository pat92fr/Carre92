from math import pi, sin, cos
 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
#from panda3d.core import KeyboardButton
from panda3d.core import *

#from direct.actor.Actor import Actor
#from direct.interval.IntervalGlobal import Sequence
#from panda3d.core import Point3
 
 
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Window
        winprops  = WindowProperties()
        winprops .setSize(1280, 720)
        base.win.requestProperties(winprops ) 
        
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
 
        self.circuitNodePath = self.loader.loadModel("/c/tmp/circuit.bam")
        self.circuitNodePath.reparentTo(self.render)
        self.circuitNodePath.setScale(1.0, 1.0, 1.0)
        self.circuitNodePath.setPos(1.5,-5,0)
        self.circuitNodePath.setHpr(0,90, 270)

        # Load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
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

        #♠self.camera.lens.setAspectRatio(1280.0 / 720.0)
        self.camLens.setFov(70)
        self.camLens.setNear(0.001)
        self.camera.setPos(0.5,0,0.2)
        self.camera.setHpr(0,0,0)
        #self.camera.setHpr(0,-20,0)
        #self.camera.reparentTo(self.box)

        # osd
        dr = self.win.makeDisplayRegion()
        dr.setSort(20)
         
        myCamera2d = NodePath(Camera('myCam2d'))
        lens = OrthographicLens()
        lens.setFilmSize(2, 2)
        lens.setNearFar(-1000, 1000)
        myCamera2d.node().setLens(lens)
         
        myRender2d = NodePath('myRender2d')
        myRender2d.setDepthTest(False)
        myRender2d.setDepthWrite(False)
        myCamera2d.reparentTo(myRender2d)
        dr.setCamera(myCamera2d)

        self.taskMgr.add(self.move_task, 'modeTask')
 
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
        
    # Define a procedure to move the camera.
#    def spinCameraTask(self, task):
#        angleDegrees = task.time * 6.0
#        angleRadians = angleDegrees * (pi / 180.0)
#        distance = 10
#        self.camera.setPos(distance * sin(angleRadians), -distance * cos(angleRadians), 2)
#        self.camera.setHpr(angleDegrees, 0, 0)
#        return Task.cont
 
app = MyApp()
app.run()
