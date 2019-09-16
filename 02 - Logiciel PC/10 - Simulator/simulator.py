## PARAMETERS #################################################################

autopilot_Kp = 8.0 # autopilot line position (prediction) to steering angle
autopilot_speed = 0.4

## GLOBALS ########################################################################

### https://github.com/jlevy44/UnrealAI/blob/master/CarAI/joshua_work/game/src/simulation.py

import numpy as np
import cv2
from keras import models
from keras.models import load_model
import my_constants as consts
import my_parameters as params

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
		winprops .setSize(1280, 720)
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
		self.circuitMaterial = Material()
		self.circuitMaterial.setShininess(0.5) #Make this material shiny
		self.circuitMaterial.setSpecular(LColor(255,255,0,0))
		self.circuitMaterial.setAmbient((0, 1, 1, 1)) 

        # load circuit model
        
		self.solNodePath = self.loader.loadModel("/c/tmp/media/sol.bam")
		self.lignenoireNodePath = self.loader.loadModel("/c/tmp/media/lignenoire.bam")
		self.ligneblancheNodePath = self.loader.loadModel("/c/tmp/media/ligneblanche.bam")
		self.bordureNodePath = self.loader.loadModel("/c/tmp/media/bordure.bam")
		self.archNodePath = self.loader.loadModel("/c/tmp/media/arch.bam")
        
		# print(str(TextureStage.getDefault()))
		# print(str(self.solNodePath.findAllTextureStages()))
		# print(str(self.solNodePath.findTextureStage('*')))
		# print(str(self.solNodePath.findAllTextures()))


		# replace base texture for sol
		tex1 = loader.loadTexture('/c/tmp/media/sol_toulouse.jpg')
		ts1 = self.solNodePath.findTextureStage('0')
		self.solNodePath.setTexture(ts1, tex1, 1)
		# add texture for illumination of sol
		tex2 = loader.loadTexture('/c/tmp/media/sol_shadow.png')
		ts2 = TextureStage('solTS')
		ts2.setMode(TextureStage.MModulate)
		ts2.setTexcoordName('0')
		ts2.setColor(LColor(1,1,1,1))
		self.solNodePath.setTexture(ts2, tex2)

		# add texture for illumination of bordure
		tex3 = loader.loadTexture('/c/tmp/media/side_shadow.png')
		ts3 = TextureStage('sideTS')
		ts3.setMode(TextureStage.MModulate)
		ts3.setTexcoordName('0')
		ts3.setColor(LColor(1,1,1,1))
		self.bordureNodePath.setTexture(ts3, tex3)


		print(str(self.solNodePath.findAllTextureStages()))
		print(str(self.solNodePath.findTextureStage('0')))
		print(str(self.solNodePath.findTextureStage('solTS')))
		print(str(self.solNodePath.findAllTextures()))

      
    
		self.circuitNodePath = NodePath('circuit')
		self.solNodePath.reparentTo(self.circuitNodePath)
		self.lignenoireNodePath.reparentTo(self.circuitNodePath)
		self.ligneblancheNodePath.reparentTo(self.circuitNodePath)
		self.bordureNodePath.reparentTo(self.circuitNodePath)
		self.archNodePath.reparentTo(self.circuitNodePath)

		self.circuitNodePath.reparentTo(self.render)
		self.circuitNodePath.setScale(1.0, 1.0, 1.0)
		self.circuitNodePath.setPos(1.0,-5,0)
		self.circuitNodePath.setHpr(0,90, 270)

#       self.circuitNodePath.setMaterial(self.circuitMaterial)



        # load the environment model.
		self.scene = self.loader.loadModel("models/environment")
		self.scene.reparentTo(self.render)
		self.scene.setScale(1.25, 1.25, 1.25)
		self.scene.setPos(0,0, -0.1)
 
        # Load the environment model.
		self.car = self.loader.loadModel("models/box")
		self.car.reparentTo(self.render)
		self.car.setScale(1.0, 1.0, 1.0)
		self.car.setPos(0, 40, 0.0)

        # Lights
		render.clearLight()

		alight = AmbientLight('ambientLight')
		alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
		alightNP = render.attachNewNode(alight)
		render.setLight(alightNP)

		dlight = DirectionalLight('directionalLight')
		dlight.setDirection(Vec3(1, 1, -1))
		dlight.setColorTemperature(6500)
		dlightNP = render.attachNewNode(dlight)
		render.setLight(dlightNP)

		s1light = PointLight('spot1Light')
		s1light.setColor(Vec4(1.0, 1.0, 1.0, 1.0))
		s1light.setPoint((0, 0, 0.5))
		s1light.setMaxDistance(4.0)
		s1light.setAttenuation((0.1,0.01,0.001))
		s1lightNP = render.attachNewNode(s1light)
		render.setLight(s1lightNP)
		#self.circuitNodePath.setLight(s1lightNP)

		# camera
		self.camLens.setFov(80)
		self.camLens.setNear(0.01)
		self.camera.setPos(0.0,0.1,0.15)
		self.camera.setHpr(0,0,0)
		self.camera.setHpr(0,-13,0)
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
		self.recording_button = KeyboardButton.ascii_key('r')
		self.recording = False
		self.autopilot_button = KeyboardButton.ascii_key('a')
		self.humanpilot_button = KeyboardButton.ascii_key('m')
		self.autopilot = True

		self.autopilot_dir = 0.0 
		self.direction = 0.0

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
		    print('quit')
		if is_down(self.recording_button):
		    self.recording  = True
		    print('recording dataset')
		if is_down(self.autopilot_button):
		    self.autopilot  = True
		    print('autopilot mode')
		if is_down(self.humanpilot_button):
		    self.autopilot  = False
		    print('manual mode')

		# if gamepad detected in human mode
		if self.gamepad and not self.autopilot:
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

		if self.autopilot:
		    self.direction = self.direction*0.8 + 0.2*self.autopilot_dir
		    self.throttle = autopilot_speed

		    # move
		    #y_delta = self.throttle * 1000.0 * task.getDt()
		    #w_delta = self.direction * 10000.0 * task.getDt()
		    y_delta = self.throttle * 1000.0 * task.getDt()
		    w_delta = self.direction * 10000.0 * task.getDt()
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

## MAIN ########################################################################

# open model
print("Load model from disk ...")
model = load_model("model/model.h5")
model.summary()
print("Done.")
try:
    mkdir(root_dir+'/'+dataset_dir)
except FileExistsError:
    pass
dataset_file = open(root_dir+'/'+dataset_dir+'/'+'dataset.txt',  'w')
# init game
app = MyApp()
counter = 0
record_counter = 0
while not app.quit:
    # game tick
    taskMgr.step()
    # get picture for CNN
    frame_orign = app.get_camera_image()
    # supress alpha channel
    frame_orign = frame_orign[:, :, 0:3] 
    #resize to CNN input format
    frame_orign = cv2.resize(frame_orign, (160, 90),   interpolation = cv2.INTER_AREA)
    # smotth and gray scale
    frame = cv2.blur(frame_orign,params.blur_kernel)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # reshape for CNN
    frame = frame.reshape(90,160,1)
    # use CNN
    yprediction = model.predict(frame.reshape(1,90,160,1)).item(0)
    ###print(str(counter) + " aiDIR:" + str(yprediction))
    # push steering from CNN to game
    app.autopilot_dir = -yprediction*autopilot_Kp
    # dataset recording
    if app.recording and counter != 0: #and not app.autopilot : # first frame buffer is empty, skip it!
        filename = dataset_dir + '/render_' + str(record_counter) + '.jpg'
        cv2.imwrite(root_dir + '/' + filename, frame_orign) 
        dataset_file.write(filename +';' + str(int(128.0-app.direction*127.0*1.4)) + ';' + str(int(app.throttle*127.0*1.4+128.0)) + '\n') # *1.4 gain
        dataset_file.flush()
        record_counter += 1

    counter += 1

dataset_file.close()
print('m:' + str(record_counter))

    
    

