from threading import Thread
import time
import sys
import cv2
import serial
import socket
import numpy as np

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=160, display_height=90, framerate=60, flip_method=2) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

class buggyServer:

    def __init__(self):

        # control 
        self.pid_camera_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0, 'error':0.0, 'last_error':0.0, 'next_error':0.0 }
        self.pid_wall_avoiding_params = {'kp':0.0, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.pid_wall_following_params = {'kp':0.01, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.filtrage_lidars = {'lid_filtree':0, 'lid_nb_samples_consecutifs':0, 'lid_nb_erreurs_consecutifs':0, 'lid_somme_samples':0, \
                                'lig_filtree':0, 'lig_nb_samples_consecutifs':0, 'lig_nb_erreurs_consecutifs':0, 'lig_somme_samples':0, \
                                'alpha':0.8 } 
        self.lig_nb_samples_consecutifs = 0
        self.wall_trheshold_min = 40
        self.wall_trheshold_max = 160
        self.wall_distance_middle = 90

        self_direction = 128
        self_throttle = 128
        self.mode = 0
        # On met -1 pour qu'au premier calcul des pids, les Lidars ne soient pas pris en compte, car nous n'avons pas encore d'acquisition
        self.lidar_distance_gauche = -1
        self.lidar_distance_droit = -1
        self.telemetry_speed = 0
        self.telemetry_manual_dir = 0
        self.telemetry_manual_thr = 0
        self.telemetry_auto_dir = 0
        self.telemetry_auto_thr = 0
        self.telemetry_mode = 0  

        # global state
        self.run_ai = True

        self.s = 0
        self.model = 0
        self.cap_send = 0
        self.out_send = 0

        self.ia_started = False
        self.wall_following_started = False
        self.screenshot = False

        self.rec_width = 0
        self.rec_height = 0

        #self.pc_IP ='10.42.0.188' #PC Section Robotique !
        self.pc_IP ='10.42.0.112' #PC HP portable Patrick !
        #self.pc_IP ='192.168.1.34' #PC Home Patrick !
        #self.pc_IP ='192.168.1.10' #PC Home Patrick !
        #self.pc_IP ='10.0.10.50' #test card


        #serial port for STM32 communication 
        self.port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=3.0)

        self.y_1 = 0
        self.y_2 = 0

        self.image = 0
        self.mode = "hires"

        if self.mode == "hires":

            self.rec_width = 1280
            self.rec_height = 720   

            self.new_width = 160
            self.new_height = 90

        if self.mode == "lores":

            self.rec_width = 1280
            self.rec_height = 720

            self.new_width = 160
            self.new_height = 90


        self.init_socket()
        self.init_cam()


    def init_socket(self):

        print("Init Socket...")

        HOST = ''  # Standard loopback interface address (localhost)
        PORT = 6000         # Port to listen on (non-privileged ports are > 1023)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.s:
            self.s.bind((HOST, PORT))
            print('listening...')
            self.s.listen()
            self.conn, addr = self.s.accept()
            print('self.connected by', addr)


    def init_wall_following(self):

        self.wall_following_started = True


    def init_IA(self):

        from keras.models import load_model
        import h5py

        # load model
        self.model = load_model('bitume_model.h5')
        self.model.summary()

        self.ia_started = True

    def init_cam(self):

        print("Init Camera...")

	# create gstreamer capture pipeline
        self.cap_send = cv2.VideoCapture(gstreamer_pipeline(capture_width=self.rec_width, capture_height=self.rec_height, display_width=self.new_width, display_height=self.new_height, framerate=60, flip_method=2), cv2.CAP_GSTREAMER)
        return_value, self.image = self.cap_send.read()
        assert(self.image.shape == (self.new_height,self.new_width,3))

        # init gstreamer broadcast pipeline
        self.out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=200 ! queue ! rtph264pay config-inerval=1 ! udpsink host='+self.pc_IP+' port=5500 sync=false',cv2.CAP_GSTREAMER,0, 20, (self.new_width,self.new_height), True)

    def image_processing(self):
        # parameters
        crop_height = 32
        height_start_1 = 0 
        height_end_1 = height_start_1+crop_height 
        height_start_2 = height_end_1
        height_end_2 = height_start_2+crop_height 

        # process image before IA

        # clean-up and grayscale
        image_smoothed= cv2.blur(self.image,(3,3))
        #########image_gray = cv2.cvtColor(image_smoothed, cv2.COLOR_RGB2GRAY)

        # crop lower and middle frames
        im_croped_1 = image_smoothed[self.new_height-height_end_1:self.new_height-height_start_1,:self.new_width]
        assert(im_croped_1.shape == (crop_height,self.new_width,3))
        im_croped_2 = image_smoothed[self.new_height-height_end_2:self.new_height-height_start_2,:self.new_width]
        assert(im_croped_2.shape == (crop_height,self.new_width,3))
        
        if self.screenshot:
            self.screenshot = False
            #### take screenshot here im_croped_2

        # CNN processing
        x_1 = im_croped_1.reshape(crop_height,self.new_width,3)/255.0
        x_2 = im_croped_2.reshape(crop_height,self.new_width,3)/255.0
        x = np.array([x_1, x_2])
        y = self.model.predict(x)
        self.y_1, self.y_2 = y.item(0), y.item(1)
        ###print(str(self.y_1))
        ###print(str(self.y_2))
        
        # draw line on image
        pt1_x = int((self.y_1+1.0)/2.0*self.new_width)
        pt1_y = int((self.new_height-height_start_1-(crop_height/2)))
        pt2_x = int((self.y_2+1.0)/2.0*self.new_width)
        pt2_y = int((self.new_height-height_end_2+(crop_height/2)))
        ###print ('coordonées : pt1({}, {}), pt2({},{})'.format(pt1_x,pt1_y,pt2_x,pt2_y))
        cv2.line(self.image,(pt1_x,pt1_y),(pt2_x,pt2_y),(255,0,0),5)

    def lidars_filtrage(valeur_courante_droit, valeur_courante_gauche):
        retour = 0
        
        # On recalcul le filtrage que si le lidar nous a donné une valeur
        # on tolere 5 erreurs consecutives, pas plus
        if valeur_courante_droit == -1:
            self.filtrage_lidars['lid_nb_erreurs_consecutifs'] = self.filtrage_lidars['lid_nb_erreurs_consecutifs'] + 1
            # Si trop d'erreurs consécutives, alors on réinitialise le filtrage
            if self.filtrage_lidars['lid_nb_erreurs_consecutifs'] == 6:
                self.filtrage_lidars['lid_nb_samples_consecutifs'] = 0
        else:
            self.filtrage_lidars['lid_nb_samples_consecutifs'] = self.filtrage_lidars['lid_nb_samples_consecutifs'] + 1
            self.filtrage_lidars['lid_nb_erreurs_consecutifs'] = 0
        
        if self.filtrage_lidars['lid_nb_erreurs_consecutifs'] < 5:
            # Si la valeur que l'on vient d'acquerrir est differente de -1 alors on calcule le filtrage
            if valeur_courante_droit != -1:
                if self.filtrage_lidars['lid_nb_samples_consecutifs'] < 10:
                    # Pour les 10 premiers echantillons, on fait une moyenne
                    self.filtrage_lidars['lid_somme_samples'] = self.filtrage_lidars['lid_somme_samples'] + valeur_courante_droit
                    self.filtrage_lidars['lid_filtree'] = self.filtrage_lidars['lid_somme_samples'] / self.filtrage_lidars['lid_nb_samples_consecutifs']
                else:
                    # Ensuite un filtrage expo
                    self.filtrage_lidars['lid_filtree'] = self.filtrage_lidars['alpha']*valeur_courante_droit + (1-self.filtrage_lidars['alpha'])*self.filtrage_lidars['lid_filtree']
            # Sinon, on ne change pas la valeur filtrée
        else:
            # sinon on le reinitialise
            self.filtrage_lidars['lid_nb_samples_consecutifs'] = 0
            self.filtrage_lidars['lid_filtree'] = -1
            retour = -1
                
        # On recalcul le filtrage que si le lidar nous a donné une valeur
        # on tolere 5 erreurs consecutives, pas plus
        if valeur_courante_gauche == -1:
            self.filtrage_lidars['lig_nb_erreurs_consecutifs'] = self.filtrage_lidars['lig_nb_erreurs_consecutifs'] + 1
            # Si trop d'erreurs consécutives, alors on réinitialise le filtrage
            if self.filtrage_lidars['lig_nb_erreurs_consecutifs'] == 6:
                self.filtrage_lidars['lig_nb_samples_consecutifs'] = 0
        else:
            self.filtrage_lidars['lig_nb_samples_consecutifs'] = self.filtrage_lidars['lig_nb_samples_consecutifs'] + 1
            self.filtrage_lidars['lig_nb_erreurs_consecutifs'] = 0
        
        if self.filtrage_lidars['lig_nb_erreurs_consecutifs'] < 5:
            # Si la valeur que l'on vient d'acquerrir est differente de -1 alors on calcule le filtrage
            if valeur_courante_droit != -1:
                if self.filtrage_lidars['lig_nb_samples_consecutifs'] < 10:
                    # Pour les 10 premiers echantillons, on fait une moyenne
                    self.filtrage_lidars['lig_somme_samples'] = self.filtrage_lidars['lig_somme_samples'] + valeur_courante_gauche
                    self.filtrage_lidars['lig_filtree'] = self.filtrage_lidars['lig_somme_samples'] / self.filtrage_lidars['lig_nb_samples_consecutifs']
                else:
                    # Ensuite un filtrage expo
                    self.filtrage_lidars['lig_filtree'] = self.filtrage_lidars['alpha']*valeur_courante_gauche + (1-self.filtrage_lidars['alpha'])*self.filtrage_lidars['lig_filtree']
            # Sinon, on ne change pas la valeur filtrée
        else:
            # sinon on le reinitialise
            self.filtrage_lidars['lig_nb_samples_consecutifs'] = 0
            self.filtrage_lidars['lig_filtree'] = -1
            retour = -1

        return retour

    def compute_pid_walls(pid_param, droit, gauche)
        pid_param['last_error'] = pid_param['error']
        pid_param['error'] = droit - gauche
        pid = pid_param['error'] * pid_param['kp'] + (pid_param['error']-pid_param['last_error']) * pid_param['kd'] 

    return pid

    def buggyservice(self):

        # game loop
        last_time = time.time()
        frame_counter = 0
        fps = 0
        y_byte = 0

        while True:

            while(True):
                value = 2
                self.conn.send(fps.to_bytes(2, byteorder='big') + y_byte.to_bytes(2, byteorder='big'))
                command = self.conn.recv(1024)

                if (command == b's'):
                    print("Screenshot!")
                    self.screenshot = True

                if (command == b'i'):
                    if self.ia_started:
                        print("IA already stared")
                    else:
                        print("initializing IA")
                        self.init_IA()

                if (command == b'w'):
                    if self.wall_following_started:
                        print("Wall following already stared")
                    else:

                        print("initializing Wall following")
                        self.init_wall_following()

                if (command == b'q'):
                    print("all done, quitting !")
                    self.s.close()
                    self.cap_send.release()
                    self.out_send.release()
                    return

                return_value, self.image = self.cap_send.read()
                assert(self.image.shape == (self.new_height,self.new_width,3))

                self.direction = 128
                if self.ia_started:
                    self.image_processing()
                    
                    #pid management
                    # On laisse ce PID comme ca, car il est différent
                    self.pid_camera_params['last_error'] = self.pid_camera_params['error']
                    self.pid_camera_params['error'] = self.pid_camera_params['error']*0.5 + 0.5*self.y_1
                    self.pid_camera_params['next_error'] = self.pid_camera_params['next_error']*0.5 + 0.5*self.y_2
                    pid_ai = self.pid_camera_params['error'] * self.pid_camera_params['kp'] + (self.pid_camera_params['error']-self.pid_camera_params['last_error']) * self.pid_camera_params['kd'] + (self.pid_camera_params['next_error']-self.pid_camera_params['error']) * self.pid_camera_params['kd2'] 
                    if pid_ai>1.0:
                        pid_ai=1.0
                    if pid_ai<-1.0:
                        pid_ai=-1.0

                    # direction & throttle control
                    self.direction = int(((pid+1.0)/2.0*256.0))
                    self.throttle = 128
                    self.mode = 1
                    
                if self.wall_following_started:
                    # direction & throttle control
                    if (self.filtrage_lidars['lid_filtree'] != -1) and (self.filtrage_lidars['lig_filtree'] != -1):
                        pid_walls_following = compute_pid_walls(self.pid_wall_following_params, self.filtrage_lidars['lid_filtree'], self.filtrage_lidars['lig_filtree'])
                    elif (self.filtrage_lidars['lid_filtree'] == -1) and (self.filtrage_lidars['lig_filtree'] != -1):
                        pid_walls_following = compute_pid_walls(self.pid_wall_following_params, 86, self.filtrage_lidars['lig_filtree'])
                    elif (self.filtrage_lidars['lid_filtree'] != -1) and (self.filtrage_lidars['lig_filtree'] == -1):
                        pid_walls_following = compute_pid_walls(self.pid_wall_following_params, self.filtrage_lidars['lid_filtree'], 86)
                    elif (self.filtrage_lidars['lid_filtree'] == -1) and (self.filtrage_lidars['lig_filtree'] == -1):
                        pid_walls_following = compute_pid_walls(self.pid_wall_following_params, 86, 86)
                    print("erreur:" + str(self.pid_wall_following_params['error']) + " PID:" + str(pid_walls_following))
                    if pid_walls_following>1.0:
                        pid_walls_following=1.0
                    if pid_walls_following<-1.0:
                        pid_walls_following=-1.0

                if self.wall_avoiding_started:
                    # direction & throttle control
                    if (self.filtrage_lidars['lid_filtree'] != -1) and (self.filtrage_lidars['lig_filtree'] != -1):
                        pid_walls_avoiding = compute_pid_walls(self.pid_wall_avoiding_params, self.filtrage_lidars['lid_filtree'], self.filtrage_lidars['lig_filtree'])
                    elif (self.filtrage_lidars['lid_filtree'] == -1) and (self.filtrage_lidars['lig_filtree'] != -1):
                        pid_walls_avoiding = compute_pid_walls(self.pid_wall_avoiding_params, 86, self.filtrage_lidars['lig_filtree'])
                    elif (self.filtrage_lidars['lid_filtree'] != -1) and (self.filtrage_lidars['lig_filtree'] == -1):
                        pid_walls_avoiding = compute_pid_walls(self.pid_wall_avoiding_params, self.filtrage_lidars['lid_filtree'], 86)
                    elif (self.filtrage_lidars['lid_filtree'] == -1) and (self.filtrage_lidars['lig_filtree'] == -1):
                        pid_walls_avoiding = compute_pid_walls(self.pid_wall_avoiding_params, 86, 86)
                    print("erreur:" + str(self.pid_wall_avoiding_params['error']) + " PID:" + str(pid_walls_avoiding))
                    if pid_walls_avoiding>1.0:
                        pid_walls_avoiding=1.0
                    if pid_walls_avoiding<-1.0:
                        pid_walls_avoiding=-1.0
                    self.direction = int(((pid+1.0)/2.0*256.0))
                    self.throttle = 128
                    self.mode = 1

                # Automate
                if(self.lidar_distance_droit != -1) and (self.lidar_distance_droit < self.wall_trheshold_min):
                    # pid mur agressif
                    self.direction = int(128 + pid_walls_avoiding * 128)    
                    self.throttle = 128
                    self.mode = 1
                elif if(self.lidar_distance_gauche != -1) and (self.lidar_distance_gauche < self.wall_trheshold_min):
                    # pid mur agressif
                    self.direction = int(128 + pid_walls_avoiding * 128)    
                    self.throttle = 128
                    self.mode = 1
                elif (self.lidar_distance_droit >= self.wall_trheshold_min) and (self.lidar_distance_droit < self.wall_trheshold_max) \
                and (self.lidar_distance_gauche >= self.wall_trheshold_min) and (self.lidar_distance_gauche < self.wall_trheshold_max):
                    # pid mur wall following
                    self.direction = int(128 + pid_walls_following * 128)    
                    self.throttle = 128
                    self.mode = 1
                elif (self.lidar_distance_gauche >= self.wall_trheshold_min) and (self.lidar_distance_gauche < self.wall_trheshold_max):
                    # pid mur wall following avec lid = 86 cm
                    pid = compute_pid_walls(self.pid_wall_following_params, 86, self.filtrage_lidars['lig_filtree'])
                    self.direction = int(128 + pid * 128)    
                    self.throttle = 128
                    self.mode = 1
                elif (self.lidar_distance_droit >= self.wall_trheshold_min) and (self.lidar_distance_droit < self.wall_trheshold_max):
                    # pid mur wall following avec lig = 86 cm
                    pid = compute_pid_walls(self.pid_wall_following_params, self.filtrage_lidars['lid_filtree'], 86)
                    self.direction = int(128 + pid * 128)    
                    self.throttle = 128
                    self.mode = 1
                else:	
                    # si j'ai les deux à -1, je vais tout droit
                    self.direction = 128
                    self.throttle = 128
                    self.mode = 0

                ###print("fps:" + str(fps) + "   error:" + str(self.y_1) + "   pid:" + str(pid) + "   servo:" + str(direction))
                self.port.write("{:d};".format(self.direction).encode('ascii') + "{:d};".format(self.throttle).encode('ascii') + "{:d}\r\n".format(self.mode).encode('ascii'))
                serial_line = self.port.readline()
                ###print(str(serial_line))
                fields = serial_line.decode('ascii').split(';')
                if(len(fields)>=8):
                   self.lidar_distance_gauche = int(fields[0])
                   self.lidar_distance_droit = int(fields[1])
                   self.telemetry_speed = int(fields[2])
                   self.telemetry_manual_dir = int(fields[3])
                   self.telemetry_manual_thr = int(fields[4])
                   self.telemetry_auto_dir = int(fields[5])
                   self.telemetry_auto_thr = int(fields[6])
                   self.telemetry_mode = int(fields[7])   
                   ##print("LiG:" + str(self.lidar_distance_gauche)+" LiD:" + str(self.lidar_distance_droit))
                # Filtrage des valeurs des lidars
                if lidars_filtrage(self.lidar_distance_droit, self.lidar_distance_gauche) == -1:
                    print("Un des lidars n'a pas pu remonter de valeur")

                self.out_send.write(self.image)

                #fps counter
                frame_counter += 1
                if time.time()>=last_time+1.0:
                    last_time=time.time()
                    fps = frame_counter
                    frame_counter = 0

def main():

    serv = buggyServer()
    serv.buggyservice()

if __name__ == "__main__":
    main()
