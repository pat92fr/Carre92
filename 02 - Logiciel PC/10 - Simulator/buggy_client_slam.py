#-*- coding: utf-8 -*-




########################
# pip install numpy
# pip install matplotlib
########################

# Import
########
import sys
import cv2
import os
import math
import random
import time
import socket
import datetime
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from matplotlib.widgets import Button
from PIL import Image

class occupancy_map:
    def __init__(self,size,scale):
        self.carto_size = size
        self.carto = np.zeros((self.carto_size,self.carto_size), dtype=np.uint8)
        self.carto.fill(125)
        self.carto_offset_x = int(size/2)
        self.carto_offset_y = int(size/2)
        self.carto_offset = np.array([self.carto_offset_x,self.carto_offset_y])
        self.carto_scale = scale

    def load(self,filename):
        img = Image.open(filename)
        self.carto = np.array(img)
        self.carto = cv2.GaussianBlur(self.carto,(5,5),0)
        self.carto = cv2.GaussianBlur(self.carto,(5,5),0)

    def plot(self):
        plt.imshow(self.carto)
        plt.show()

    def save(self):
        self.carto_blur = cv2.GaussianBlur(self.carto,(3,3),0)
        img = Image.fromarray(self.carto_blur, 'L')
        img.save('occupancy_map.png')

    def ray(self,origin,end,collision=True):
        rescaled_origin = origin*self.carto_scale+self.carto_offset
        rescaled_end = end*self.carto_scale+self.carto_offset
        cv2.line(
            self.carto,
            (int(rescaled_origin[1]),int(rescaled_origin[0])),
            (int(rescaled_end[1]),int(rescaled_end[0])),
            0,
            2)
        if collision:
            cv2.circle(
                self.carto,
                (int(rescaled_end[1]),int(rescaled_end[0])),
                3, 
                255, 
                -1)

    def valid_position(self,particle):
        x,y,a,w = particle
        valid = self.carto[x,y] == 0
        return valid



class particle_filter:
    def __init__(self,count):
        print("PAF init...")
        self.particle_list = []
        self.particle_count = count
        print("Done!")

    def fill(self,carto):
        print("PAF fill..." + str(len(self.particle_list)))
        while len(self.particle_list) < self.particle_count:
            particle = (
                random.randrange(0,carto.carto_size,1), # pose x
                random.randrange(0,carto.carto_size,1), # pose y
                random.randrange(0,360,1), # orientation
                0.5 # weight
                )
            if carto.valid_position(particle):
                xn,yn,an,wn = particle
                collision = False
                for p in self.particle_list:
                    x,y,a,w = p
                    if x == xn and y == yn:
                        collision = True
                if not collision:
                    self.particle_list.append(particle)
                    ###print(particle)
            ###print(str(len(self.particle_list)))
        print("Done! " + str(len(self.particle_list)))

    def plot(self,carto):
        # plot particules over carto and usine line for orientation and circle for position
        carto_picture = np.copy(carto.carto)
        print(str(carto_picture.shape))
        print(str(carto_picture[0,0]))
        print(str(len(self.particle_list)))
        for p in self.particle_list:
            #print(".")
            x,y,a,w = p
            origin = np.array([x,y])
            target = np.array([x+10*math.cos(math.radians(a)),y+10*math.sin(math.radians(a))])
            cv2.line(
                carto_picture,
                (int(origin[1]),int(origin[0])),
                (int(target[1]),int(target[0])),
                128,
                1)
            cv2.circle(
                carto_picture,
                (int(origin[1]),int(origin[0])),
                3, 
                max(int(100+w*250),255), 
                -1)
        print("Drawing")
        img = Image.fromarray(carto_picture, 'L')
        img.save('particle_map.png')
        #plt.imshow(carto_picture) #np.transpose(npimg, (1, 2, 0)))
        #plt.show()
        #plt.pause(0.01)
        #print("show!")            

    def sensor_update_weight(self,observation_list,carto):
        new_particle_list = []
        for p in self.particle_list:
            px,py,pa,pw = p
            ###print(p)
            pw = 0.0
            for o in observation_list:
                od, oa, ocollision = o
                ###print(o)

                # compute target point
                tx = int(px + od * math.cos(math.radians(pa+oa)))
                ty = int(py + od * math.sin(math.radians(pa+oa)))

                # get occupancy map value
                occup = carto.carto[ty,tx]
                #print(str(occup))
                # define weight
                if ocollision:
                    # expected occ is 255 ==> w = 1.0
                    # occ < 127 ==> w = 0.0
                    pw += min(occup - 217.0,128.0)/128.0
                else:
                    # expected occ is 0 ==> w = 1.0
                    # occ = 127 ==> w = 0.0
                    pw += (127.0-max(occup,127.0))/127.0
                ##if pw > 0.25:
                ##    print(str(pw))
            new_particle_list.append( (px,py,pa,pw) )
        self.particle_list = new_particle_list  

    def normalize_weight(self):
        sum_of_weight = 0.0
        for p in self.particle_list:
            px,py,pa,pw = p
            sum_of_weight += pw
        new_particle_list = []
        for p in self.particle_list:
            px,py,pa,pw = p
            pw /= sum_of_weight
            new_particle_list.append( (px,py,pa,pw) )
        self.particle_list = new_particle_list  


    def resampling_and_motion(self,v,da,dt,carto):

        #sort by weight
        def getKey(item):
            return item[3]
        sorted(self.particle_list, key=getKey, reverse=True)

        new_particle_list = []
        # double best particles
        for p in self.particle_list[0:50]:
            px,py,pa,pw = p
            # perfect move update
            pa1 = pa + da
            px1 = int( px + v*dt * math.cos(math.radians(pa1))*carto.carto_scale )
            py1 = int( py + v*dt * math.sin(math.radians(pa1))*carto.carto_scale )
            np1 = (px1,py1,pa1,pw)
            if carto.valid_position( np1 ):
                new_particle_list.append( np1 )

            pa2 = pa + da + random.uniform(-1.0,1.0)
            dx2 = random.uniform(-0.2,0.2)
            px2 = int( px + (v*dt+dx2) * math.cos(math.radians(pa2))*carto.carto_scale )
            py2 = int( py + (v*dt+dx2) * math.sin(math.radians(pa2))*carto.carto_scale )
            np2 = (px2,py2,pa2,pw)
            if carto.valid_position( np2 ):
                new_particle_list.append( np2 )

            pa3 = pa + da + random.uniform(-1.0,1.0)
            dx3 = random.uniform(-0.2,0.2)
            px3 = int( px + (v*dt+dx3) * math.cos(math.radians(pa3))*carto.carto_scale )
            py3 = int( py + (v*dt+dx3) * math.sin(math.radians(pa3))*carto.carto_scale )
            np3 = (px3,py3,pa3,pw)
            if carto.valid_position( np3 ):
                new_particle_list.append( np3 )
        
        # copy plain particles
        for p in self.particle_list[50:280]:
            px,py,pa,pw = p
            # perfect move update
            pa0 = pa + da + random.uniform(-1.0,1.0)
            dx0 = random.uniform(-0.2,0.2)
            px0 = int( px + (v*dt+dx0) * math.cos(math.radians(pa0))*carto.carto_scale )
            py0 = int( py + (v*dt+dx0) * math.sin(math.radians(pa0))*carto.carto_scale )
            np0 = (px0,py0,pa0,pw)
            if carto.valid_position( np0 ):
                new_particle_list.append( np0 )

        #print("resampled particles:" + str(len(new_particle_list)))

        self.particle_list = new_particle_list  
        while len(new_particle_list) < self.particle_count:
            particle = (
                random.randrange(0,carto.carto_size,1), # pose x
                random.randrange(0,carto.carto_size,1), # pose y
                random.randrange(0,360,1), # orientation
                0.0 # weight
                )
            if carto.valid_position(particle):
                xn,yn,an,wn = particle
                collision = False
                for p in self.particle_list:
                    x,y,a,w = p
                    if x == xn and y == yn:
                        collision = True
                if not collision:
                    new_particle_list.append(particle)
        self.particle_list = new_particle_list  
        #print("adding particles:" + str(len(self.particle_list)))




    def refill(self,carto):
        self.fill(carto)

# Main procedure
################
def main():

    random.seed(time.time())

    # parameters
    learning = False
    particle_count = 300

    # constants
    carto_size = 4086
    carto_scale = 50

    # cartograph
    carto_filename = 'occupancy_map01.png'
    carto = occupancy_map(carto_size,carto_scale)

    # load if already done
    if not learning:
        carto.load(carto_filename)
        ##carto.plot()

    # particle filter
    paf = particle_filter(particle_count)

    if not learning:
        paf.fill(carto)
        ##paf.plot(carto)





    server_address = '192.168.1.34'
    server_port = 7001
    param_count = 16

    # Try the connection to the server
    ##################################
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((server_address, server_port))
    except:
        print('Connection #KO# to ' + server_address + ':' + str(server_port))
        sys.exit(0)

    print('Connection OK to ' + server_address + ':' + str(server_port))

    # Main while loop
    x = 0.0 #m
    y = 0.0 #m    
    last_heading = 0.0
    loop_counter = 1
    file_dataset = open('dataset.txt','w')
    while True:
    
        # Wait for length
        data = client.recv(4)
        if len(data) != 0:
            length = int(data.decode("utf-8").replace(' ',''))
            # Wait for the payload
            data = client.recv(length)
            if len(data) != 0:
                res = data.decode("utf-8").rstrip('\n').rstrip(' ').split(';')
                if len(res) != param_count:
                    print('#Error')
                    continue

                # read data
                right_lidar = float(res[2])
                left_lidar = float(res[1])
                speed = float(res[8])
                heading = float(res[15])
                delta_heading = heading-last_heading
                last_heading = heading
                #print("right_lidar:" + str(round(right_lidar,2)) + " left_lidar:" + str(round(left_lidar,2)) + " speed:" + str(round(speed,2)) + " heading:" + str(int(heading)) )


                if loop_counter>1: # we need dw
                    file_dataset.write( str(round(right_lidar,3)) +';' +str(round(left_lidar,3)) +';' +str(round(speed,4)) +';' +str(round(delta_heading,4)) + "\n")
                    file_dataset.flush()


                if learning:

                    # estimate position
                    x += speed * 1.0/60.0 * math.cos(math.radians(heading))
                    y += speed * 1.0/60.0 * math.sin(math.radians(heading))
                    print("position:(" + str(round(x,2)) + "," + str(round(y,2)) + ")" )

                    x1 = x + left_lidar * math.cos(math.radians(heading+60))
                    y1 = y + left_lidar * math.sin(math.radians(heading+60))
                    x2 = x + right_lidar * math.cos(math.radians(heading-60))
                    y2 = y + right_lidar * math.sin(math.radians(heading-60))

                    # update map
                    origin = np.array([x,y])
                    end1 = np.array([x1,y1])
                    end2 = np.array([x2,y2])

                    carto.ray(origin,end1,left_lidar<1.9)
                    carto.ray(origin,end2,right_lidar<1.9)
                    if loop_counter % (60*30) == 0:
                        carto.save()

                # elif loop_counter>1: # we need dw

                #     paf.resampling_and_motion(speed,delta_heading,1/60.0,carto)
                #     observation_list = []
                #     observation_list.append((left_lidar,60,left_lidar<1.9))
                #     observation_list.append((right_lidar,-60,right_lidar<1.9))
                #     paf.sensor_update_weight(observation_list,carto)
                #     paf.normalize_weight()

                #     if loop_counter % (60*10) == 0:
                #         paf.plot(carto)






                loop_counter += 1
                    #img = Image.fromarray(carto, 'RGB')
                    #img.save('my.png')
                    #img.show()
                    #plt.imshow(carto)
                    #plt.pause(0.001)





# Main
if __name__ == '__main__':    
    main()

# EOF

