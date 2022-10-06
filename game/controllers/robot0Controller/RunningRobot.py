from functions import *
from MazeRobot import MazeRobot
import cv2
import numpy as np

TIME_STEP = 32

class RunningRobot(MazeRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.left_wheel.setVelocity(4)
            self.right_wheel.setVelocity(4)
            
        
            img = self.right_camera.getImage()
            img = np.frombuffer(img, np.uint8).reshape((self.right_camera.getHeight(), self.right_camera.getWidth(), 4))
            cam(img)

            img2 = self.left_camera.getImage()
            img2 = np.frombuffer(img2, np.uint8).reshape((self.left_camera.getHeight(), self.left_camera.getWidth(), 4))
            cam(img2)

            #gyro_values= self.gyro.getValues()

            #round_gyro_values = [f"{num:.2f}" for num in gyro_values] #round gyro values
            #print(gyro_values)

            #x,z = get_all_values(self)
            #get_all_values(self)


            x = self.gps.getValues()[0]
            z = self.gps.getValues()[2]
            x*=100 #convert form meter to cm
            z*=100
            print(x,z)
            map_gen(self,x, z)
            """
            dist = math.sqrt((x-self.startX)**2 + (z-self.startZ)**2); # Use the distance formula to calculate distance to starting position
            #print(self.robot_pos)
            
            if move_one_tile(dist):
                self.left_wheel.setVelocity(0)
                self.right_wheel.setVelocity(0)

            else :
                self.left_wheel.setVelocity(4)
                self.right_wheel.setVelocity(4)
                """

            """dist= int((dist-int(dist))*100%10 )
            if dist >=4 and dist <=5:
                self.left_wheel.setVelocity(0)
                self.right_wheel.setVelocity(0)
                
            else:
                
                self.left_wheel.setVelocity(4)
                self.right_wheel.setVelocity(4)


            print("Distance from starting location: " , dist)
            print(self.startX,self.startZ)
            

        """


           
            image = self.color_sensor.getImage()
    
             # Get the individual RGB color channels from the pixel (0,0) 
            r = self.color_sensor.imageGetRed(image, 1, 0, 0)
            g = self.color_sensor.imageGetGreen(image, 1, 0, 0)
            b = self.color_sensor.imageGetBlue(image, 1, 0, 0)
            viewColour(self,r, g, b)
            
            #print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
            
    
            get_all_values(self)
            print(self.robot_pos)
            print(self.color_sensor_values)

