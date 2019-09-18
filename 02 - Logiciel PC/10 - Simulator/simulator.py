## PARAMETERS #################################################################

autopilot_Kp = 8.0 #8.0 # autopilot line position (prediction) to steering angle
autopilot_speed = 0.4

## GLOBALS ########################################################################

### https://github.com/jlevy44/UnrealAI/blob/master/CarAI/joshua_work/game/src/simulation.py

import numpy as np
import cv2
from keras import models
from keras.models import load_model

from direct.showbase.ShowBase import ShowBase
from direct.filter.CommonFilters import CommonFilters
from direct.gui.OnscreenText import OnscreenText
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

# Macro-like function used to reduce the amount to code needed to create the
# on screen instructions
def genLabelText(text, i):
    return OnscreenText(text=text, parent=base.a2dTopLeft, pos=(0.07, -.06 * i - 0.1),
                        fg=(1, 1, 1, 1), align=TextNode.ALeft, shadow=(0, 0, 0, 0.5), scale=.05)



class MyApp(ShowBase):
	def __init__(self):
		ShowBase.__init__(self)
        
        # OSD menu
		self.shadowText = genLabelText("s: Ground shadows", 0)
		self.publicityText = genLabelText("p: Side publicity and shadows", 1)

        # dynamic settings
		self.apply_ground_shadow = False
		self.apply_side_publicity_and_shadow = False

		#events
		self.accept("s",     self.setGroundShadow)
		self.accept("p",     self.setSidePublicityAndShadow)

        # Check video card capabilities.
		if not self.win.getGsg().getSupportsBasicShaders():
			addTitle("Bump Mapping: "
				"Video driver reports that Cg shaders are not supported.")
			return

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

		# load map
		self.load_toulouse_map()

        # make car node (no model attached)
		self.car = NodePath('car')
		self.car.reparentTo(self.render)
		self.car.setPos(0, 40, 0.0)

		# camera
		self.camLens.setFov(90)
		self.camLens.setNear(0.01)
		self.camera.setPos(0.0,0.1,0.20)
		self.camera.setHpr(0,0,0)
		self.camera.setHpr(0,-15,0)
		self.camera.reparentTo(self.car)

		# osd
		self.dr = self.win.makeDisplayRegion()
		self.dr.setSort(20)

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


	def load_toulouse_map(self):

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
		#tex1 = loader.loadTexture('/c/tmp/media/sol_NormalMap.jpg')
		#tex1 = loader.loadTexture('/c/tmp/media/sol_DisplacementMap.jpg')
		self.ts1 = self.solNodePath.findTextureStage('0')
		self.ts1.setTexcoordName('0')
		self.solNodePath.setTexture(self.ts1, tex1, 1)

		# add texture for illumination of sol
		self.sol_no_shadow_texture = loader.loadTexture('/c/tmp/media/no_shadow.png')
		self.sol_shadow_textures = [ 
			loader.loadTexture('/c/tmp/media/sol_shadow_1.png'),
			loader.loadTexture('/c/tmp/media/sol_shadow_2.png'),
			loader.loadTexture('/c/tmp/media/sol_shadow_3.png'),
			loader.loadTexture('/c/tmp/media/sol_shadow_4.png')
		]
		self.sol_shadow_texture_index = 0
		self.ts2 = TextureStage('solTS')
		self.ts2.setMode(TextureStage.MModulate)
		self.ts2.setTexcoordName('0')
		self.ts2.setColor(LColor(0,0,0,1))
		self.solNodePath.setTexture(self.ts2, self.sol_no_shadow_texture)

		# add texture for normalmap of sol
		tex4 = loader.loadTexture('/c/tmp/media/sol_NormalMap.jpg','/c/tmp/media/sol_DisplacementMap.jpg')
		ts4 = TextureStage('solTSNM')
		ts4.setMode(TextureStage.MNormalHeight)
		ts4.setTexcoordName('0')
		#self.solNodePath.setTexture(ts4, tex4)


		# replace base texture for side
		self.side_base_texture_no_publicity = loader.loadTexture('/c/tmp/media/2create_wood_0019o.jpg')
		self.side_base_texture_with_publicity = loader.loadTexture('/c/tmp/media/2create_wood_0019.jpg')
		self.ts4 = self.bordureNodePath.findTextureStage('0')
		self.ts4.setTexcoordName('0')
		self.bordureNodePath.setTexture(self.ts4, self.side_base_texture_no_publicity, 1)

		# add texture for illumination of bordure
		self.side_no_shadow_texture = loader.loadTexture('/c/tmp/media/no_shadow.png')
		self.side_shadow_texture = loader.loadTexture('/c/tmp/media/side_shadow.png')
		self.ts3 = TextureStage('sideTS')
		self.ts3.setMode(TextureStage.MModulate)
		self.ts3.setTexcoordName('0')
		self.ts3.setColor(LColor(0,0,0,1))
		self.bordureNodePath.setTexture(self.ts3, self.side_no_shadow_texture)

		# print(str(self.solNodePath.findAllTextureStages()))
		# print(str(self.solNodePath.findTextureStage('0')))
		# print(str(self.solNodePath.findTextureStage('solTS')))
		# print(str(self.solNodePath.findTextureStage('solTSNM')))
		# print(str(self.solNodePath.findAllTextures()))

		# NormalMap https://cpetry.github.io/NormalMap-Online/

		# create material
		self.lowSpecMaterial = Material()
		self.lowSpecMaterial.setSpecular((0.05, 0.05, 0.05, 1))
		self.lowSpecMaterial.setShininess(10) #Make this material shiny

		self.hiSpecMaterial = Material()
		self.hiSpecMaterial.setSpecular((0.9, 0.9, 0.9, 1))
		self.hiSpecMaterial.setShininess(80) #Make this material shiny

		#apply material
		self.solNodePath.setMaterial(self.lowSpecMaterial, 1)
		self.lignenoireNodePath.setMaterial(self.hiSpecMaterial, 1)
		self.ligneblancheNodePath.setMaterial(self.hiSpecMaterial, 1)
		self.bordureNodePath.setMaterial(self.lowSpecMaterial, 1)
		self.archNodePath.setMaterial(self.lowSpecMaterial, 1)

        #
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

        # load the environment model.
		self.scene = self.loader.loadModel("/c/tmp/media/env")
		self.scene.reparentTo(self.render)
		self.scene.setScale(100, 100, 100)
		self.scene.setPos(0,0, -0.1)
 
        # Lights
		render.clearLight()

		alight = AmbientLight('ambientLight')
		alight.setColor(Vec4(0.55, 0.55, 0.55, 1))
		alightNP = render.attachNewNode(alight)

		directionalLight = DirectionalLight('directionalLight')
		directionalLight.setDirection(Vec3(1, 1, -1))
		directionalLight.setSpecularColor((0.8, 0.8, 0.8, 1))
		directionalLight.setColorTemperature(6500)
		directionalLightNP = render.attachNewNode(directionalLight)

		plight = PointLight('spot1Light')
		plight.setColor(Vec4(1.0, 1.0, 1.0, 1.0))
		plight.setAttenuation(LVector3(0.7, 0.05, 0))
		plnp = self.render.attachNewNode(plight)
		plnp.setPos((0, 0, 1.0))

        # Tell Panda that it should generate shaders performing per-pixel
        # lighting for the room.
		self.circuitNodePath.setLight(plnp)
		self.circuitNodePath.setLight(directionalLightNP)
		#self.circuitNodePath.setShaderAuto()
		render.setLight(alightNP)

	def update_toulouse_map(self):
		if self.apply_ground_shadow:
			self.sol_shadow_texture_index += 1
			if self.sol_shadow_texture_index >= len(self.sol_shadow_textures):
				self.sol_shadow_texture_index = 0
			self.solNodePath.setTexture(self.ts2, self.sol_shadow_textures[self.sol_shadow_texture_index])

	def setGroundShadow(self):
		if self.apply_ground_shadow:
			self.apply_ground_shadow = False
			self.solNodePath.setTexture(self.ts2, self.sol_no_shadow_texture)
		else:
			self.apply_ground_shadow = True
			self.sol_shadow_texture_index = 0
			self.solNodePath.setTexture(self.ts2, self.sol_shadow_textures[self.sol_shadow_texture_index])
			self.sol_shadow_texture_index += 1

	def setSidePublicityAndShadow(self):
		if self.apply_side_publicity_and_shadow:
			self.apply_side_publicity_and_shadow = False
			self.bordureNodePath.setTexture(self.ts3, self.side_no_shadow_texture)
			self.bordureNodePath.setTexture(self.ts4, self.side_base_texture_no_publicity, 1)
		else:
			self.apply_side_publicity_and_shadow = True
			self.bordureNodePath.setTexture(self.ts3, self.side_shadow_texture)
			self.bordureNodePath.setTexture(self.ts4, self.side_base_texture_with_publicity, 1)

	def windDown(self):
        # De-initialization code goes here!
		self.quit = True
  
	def move_task(self, task):

		if task.frame % 180 == 0:
			self.update_toulouse_map()

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
		    
		    # move
		    y_delta = self.throttle * 100.0 * task.getDt()
		    w_delta = self.direction * 1000.0 * task.getDt()
		    self.car.setHpr(self.car, w_delta, 0, 0)
		    self.car.set_y(self.car, y_delta)

		if self.autopilot:
			# AI inputs
		    self.direction = self.direction*0.8 + 0.2*self.autopilot_dir
		    self.throttle = autopilot_speed

		    # move
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
    frame = cv2.blur(frame_orign,(3,3))
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

    
    

