from MazeRobot import MazeRobot
import cv2


def set_left_vel(robot: MazeRobot, v):
    robot.left_wheel.setVelocity(-v)

def set_right_vel(robot: MazeRobot, v):
    robot.right_wheel.setVelocity(-v)


def get_gps(robot: MazeRobot):
    robot.robot_pos[0] = robot.gps.getValues()[0]
    robot.robot_pos[1] = robot.gps.getValues()[2]

def get_color_sensor(robot: MazeRobot):
    robot.image = robot.color_sensor.getImage()
    robot.color_sensor_values[0] = robot.color_sensor.imageGetRed(robot.image, 1, 0, 0)
    robot.color_sensor_values[1] = robot.color_sensor.imageGetGreen(robot.image, 1, 0, 0)
    robot.color_sensor_values[2] = robot.color_sensor.imageGetBlue(robot.image, 1, 0, 0)

def get_all_values(robot: MazeRobot):
    get_gps(robot)
    get_color_sensor(robot)


def map_gen(robot: MazeRobot,x,z):
    x_axis2indx = int((abs(robot.x_dimension)+x)/12)
    z_axis2indx = int((abs(robot.z_dimension)+z)/12)
    if(x_axis2indx==0):
        x_axis2indx=2
    else:
        x_axis2indx=(4*x_axis2indx)+2
    if(z_axis2indx==0):
        z_axis2indx=2
    else:
        z_axis2indx=(4*z_axis2indx)+2
    

    if robot.map[z_axis2indx][x_axis2indx]==-1:
        if robot.color_case=="white":
            robot.map[z_axis2indx][x_axis2indx]=0
        if robot.color_case=="orange":
            robot.map[z_axis2indx][x_axis2indx]=3
    for i in robot.map:
        print(i)
    #print(robot.map)
    print("""""")
        



def move_one_tile(dic):
    #print(dic)
    dic= int( (dic-int(dic))*100%10 )
    
    if dic >=4 and dic <=5:
        return False
    else:
        return False

# Avoid holes and swamps by looking at the RBG colour of the camera
def viewColour(robot:MazeRobot, r,g,b):
    color_case=""
    
    if (r>=200) and (g>=200) and (b>=200):
        print("White")
        robot.color_case ="white"
    elif (r>=230) and (g>=200) and (g<=240) and (b>110) and (b<=160):
        print("Orange")
        robot.color_case="orange"
    elif (r<70) and (g<70) and (b<70):
        print("Black")
        robot.color_case="black"




 

def cam(img):

    
    rgb = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    rgb_copy = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    #blanc = np.zeros(img.shape[:2] , dtype='uint8')
    #blanc.fill(255)
    #blanc = cv2.cvtColor(blanc, cv2.COLOR_BGR2RGB)

    mask1 = cv2.inRange(rgb, (200, 200, 200), (255, 255,255)) #masking white
    rgb[mask1>0]=(255,0,0)
    cv2.imshow("rgb" , rgb)

    difference = cv2.subtract(rgb, rgb_copy)
    b, g, r = cv2.split(difference)
    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0:
        print("No Victim detected")

    else: 
        print("white")
        #img2 = np.zeros(mask1.shape[:2] , dtype='uint8')
        #img2 = cv2.bitwise_xor(blanc,rgb,mask=mask1)
        #img2 = cv2.bitwise_xor(blanc,rgb)
        #img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
        
        #cv2.imshow("JJ" , img2)
        
        mask2 = cv2.inRange(rgb_copy, (-50, -50, -50), (80, 80,80))
        rgb_copy[mask2>0]=(0,0,255)
        difference = cv2.subtract(rgb, rgb_copy)
        b, g, r = cv2.split(difference)
        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0:
            print("Not found")
        else: 
         cv2.imshow("img2" ,rgb_copy)
         print("victim detected")
    cv2.waitKey(1)



