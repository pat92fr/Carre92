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
from matplotlib.widgets import Button
from PIL import Image



class occupancy_map:
    def __init__(self,size,scale,ax,ay,bx,by):
        self.carto_size = size
        self.carto = np.zeros((self.carto_size,self.carto_size), dtype=np.uint8)
        self.carto.fill(125)
        self.carto_offset_x = int(size/2)
        self.carto_offset_y = int(size/2)
        self.carto_offset = np.array([self.carto_offset_x,self.carto_offset_y])
        self.carto_scale = scale
        self.ax = ax
        self.ay = ay
        self.bx = bx
        self.by = by

    def load(self,filename):
        img = Image.open(filename)
        self.carto = np.array(img)
        self.carto = cv2.GaussianBlur(self.carto,(5,5),0)
        self.carto = cv2.GaussianBlur(self.carto,(3,3),0)


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
                random.randrange(carto.ax,carto.bx,1), # pose x
                random.randrange(carto.ay,carto.by,1), # pose y
                random.uniform(0.0,360.0), # orientation
                1.0/self.particle_count # weight
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

    def plot(self,carto,observation_list,pos,counter):
        # plot particules over carto and usine line for orientation and circle for position
        carto_picture = np.copy(carto.carto)
        print(str(carto_picture.shape))
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
                10, #3, 
                min(int(50+w*self.particle_count*200),255), 
                -1)
            # for o in observation_list:
            #     od, oa, ocollision = o
            #     ###print(o)

            #     # compute target point
            #     ox = int(x + od * math.cos(math.radians(a+oa)) * carto.carto_scale )
            #     oy = int(y + od * math.sin(math.radians(a+oa)) * carto.carto_scale)

            #     op = np.array([ox,oy])
            #     cv2.line(
	           #      carto_picture,
	           #      (int(origin[1]),int(origin[0])),
	           #      (int(op[1]),int(op[0])),
	           #      100,
	           #      1)

            #     # get occupancy map value
            #     occup = carto.carto[ox,oy]
            #     #print(str(occup))
            #     # define weight
            #     if ocollision:
            #         # expected occ is 255 ==> w = 1.0
            #         # occ < 127 ==> w = 0.0
            #         opw = max(occup - 127.0,0.0)/128.0
            #     else:
            #         # expected occ is 0 ==> w = 1.0
            #         # occ = 124 ==> w = 0.0
            #         opw = (124.0-min(occup,124.0))/124.0          
            #     cv2.circle(
	           #      carto_picture,
	           #      (int(op[1]),int(op[0])),
	           #      2, 
	           #      int(20+opw*200), 
	           #      1)   
        for p in pos:
            x,y = p
            px = int(x* carto.carto_scale)+1000 #carto.carto_offset_x
            py = int(y* carto.carto_scale)+1000 #carto.carto_offset_y
            p = [px,py]
            cv2.circle(
				carto_picture,
				(int(p[1]),int(p[0])),
				2, 
				255, 
				-1)  
        print("Drawing")
        img = Image.fromarray(carto_picture, 'L')
        img.save(counter + 'particle_map.png')
        #plt.imshow(carto_picture) #np.transpose(npimg, (1, 2, 0)))
        #plt.show()
        #plt.pause(0.001)
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
                tx = int(px + od * math.cos(math.radians(pa+oa)) * carto.carto_scale )
                ty = int(py + od * math.sin(math.radians(pa+oa)) * carto.carto_scale)

                # get occupancy map value
                occup = carto.carto[tx,ty]
                #print(str(occup))
                # define weight
                if ocollision:
                    # expected occ is 255 ==> w = 1.0
                    # occ < 127 ==> w = 0.0
                    pw += max(occup - 127.0,0.0)/128.0
                else:
                    # expected occ is 0 ==> w = 1.0
                    # occ = 124 ==> w = 0.0
                    pw += (124.0-min(occup,124.0))/124.0
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

        w = []
        for p in self.particle_list:
            px,py,pa,pw = p
            w.append(pw)
        w = np.array(w)

        new_particle_list_index = np.random.choice(
            len(self.particle_list), 
            self.particle_count-int(self.particle_count*1/10), 
            p=w)

        # then move with rand error
        new_particle_list = []
        for i in new_particle_list_index:
            px,py,pa,pw = self.particle_list[i]
            pa0 = pa + da + random.uniform(-0.1,0.1)
            dx0 = random.uniform(-0.002,0.002)
            px0 = int( px + (v*dt+dx0) * math.cos(math.radians(pa0)) * carto.carto_scale )
            py0 = int( py + (v*dt+dx0) * math.sin(math.radians(pa0)) * carto.carto_scale )
            np0 = (px0,py0,pa0,pw)
            if carto.valid_position( np0 ):
                new_particle_list.append( np0 )
        self.particle_list = new_particle_list

        #add random particles
        self.fill(carto)


random.seed(time.time())

# parameters
learning = False
particle_count = 1000

# constants
carto_size = 4086
carto_scale = 50

# cartograph
carto_filename = 'occupancy_map01.png'
carto = occupancy_map(carto_size,carto_scale,500,1800,3000,2100)

# load if already done
if not learning:
    carto.load(carto_filename)
    ##carto.plot()

# particle filter
paf = particle_filter(particle_count)

if not learning:
    paf.fill(carto)
    paf.plot(carto,[],[],"init_")

# read file
print("Loading dataset file...")
file_dataset = open("dataset.txt", "r")
dataset_content = file_dataset.read()
file_dataset.close()
print("Done.")

# parse dataset  
print("Parsing dataset file...")
lines = dataset_content.splitlines()
m = len(lines)
print(" m=" + str(m) + " entries")
print("Done.")

# for each entry (picture,linepos) of current dataset
print("Building dataset...")
counter = 0
x = 0.0
y = 0.0
w = 0.0
pos = []
for l in lines[1:-1]:
	fields = l.split(';')
	right_lidar = float(fields[0])
	left_lidar = float(fields[1])
	speed = float(fields[2])
	delta_heading = float(fields[3])
	print(str(right_lidar) + ' ' + str(left_lidar) + ' ' + str(speed) + ' ' + str(delta_heading) )


	w += delta_heading
	x += speed/60.0 * math.cos(math.radians(w))
	y += speed/60.0 * math.sin(math.radians(w))
	pos.append( (x,y) )

	paf.resampling_and_motion(speed,delta_heading,1/60.0,carto)
	observation_list = []
	observation_list.append((left_lidar,60,left_lidar<1.9))
	observation_list.append((right_lidar,-60,right_lidar<1.9))
	paf.sensor_update_weight(observation_list,carto)
	paf.normalize_weight()

	if counter % 60 == 0:
	#if True:
		paf.plot(carto,observation_list,pos,str(counter)+'_')
	counter += 1

print("Done.")