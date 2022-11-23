import cv2
import numpy as np
from MazeRobot import MazeRobot
from RCJRVision import RCJRVision



def move(robot = MazeRobot):
    set_left_vel(robot, 3)
    set_right_vel(robot, -3)

def stop(robot = MazeRobot):
    set_left_vel(robot, 0)
    set_right_vel(robot, 0)

def set_left_vel(robot: MazeRobot, v):
    robot.left_wheel.setVelocity(v)

def set_right_vel(robot: MazeRobot, v):
    robot.right_wheel.setVelocity(v)


def get_gps(robot: MazeRobot):
    robot.robot_pos[0] = robot.gps.getValues()[0]
    robot.robot_pos[1] = robot.gps.getValues()[2]

def get_color_sensor(robot: MazeRobot):
    robot.image = robot.color_sensor.getImage()
    robot.color_sensor_values[0] = robot.color_sensor.imageGetRed(robot.image, 1, 0, 0)
    robot.color_sensor_values[1] = robot.color_sensor.imageGetGreen(robot.image, 1, 0, 0)
    robot.color_sensor_values[2] = robot.color_sensor.imageGetBlue(robot.image, 1, 0, 0)
    r= robot.color_sensor_values[0]
    g=robot.color_sensor_values[1]
    b=robot.color_sensor_values[2]

    if (r>=200) and (g>=200) and (b>=200):
        print("White")
        robot.color_case ="white"
    elif (r>=230) and (g>=200) and (g<=240) and (b>110) and (b<=160):
        print("Orange")
        robot.color_case="orange"
    elif (r<70) and (g<70) and (b<70):
        print("Black")
        robot.color_case="black"

def get_all_values(robot: MazeRobot):
    get_gps(robot)
    get_color_sensor(robot)

def color2num(color):
    if color == "white": return 0
    elif color == "orange": return 3
    elif color == "black" : return 2
    else: return 0

def map_updeter(robot: MazeRobot,x,z):

    x_axis2indx = (abs(robot.x_dimension)+x)/12 
    z_axis2indx = (abs(robot.z_dimension)+z)/12

    #print(x_axis2indx-int(x_axis2indx))
    #print(z_axis2indx-int(z_axis2indx))
    
    if(x_axis2indx-int(x_axis2indx) >=0.47  and x_axis2indx-int(x_axis2indx) <=0.53 and  z_axis2indx-int(z_axis2indx) >= 0.47 and z_axis2indx-int(z_axis2indx) <=0.53 ):
        stop(robot)
        print("stooop")

        x_axis2indx=int(x_axis2indx)
        z_axis2indx=int(z_axis2indx)

        x_center=(4*x_axis2indx)+2
        z_center=(4*z_axis2indx)+2

        if robot.map[z_center][x_center]==-1:

            #//////////////Fill Corners////////////////////
            dz= [-1,-1,1,1]
            dx= [-1,1,-1,1]
            if not robot.start_point:
                corners_value=5
                robot.start_point=1
            else : corners_value= color2num(robot.color_case)
            for i in range(0, 4):
                robot.map[z_center+dz[i]][x_center+dx[i]]=corners_value

            #//////////Fill center and sides//////////////
            robot.map[z_center][x_center]=0
            dz= [-1,0,1,0]
            dx= [0,-1,0,1]
            for i in range(0, 4):
                robot.map[z_center+dz[i]][x_center+dx[i]]=0

            #//////////print the matrix/////////////////
            for i in robot.map:
                print(i)
        


def viewColour(robot:MazeRobot, r,g,b):
    color_case=""
    print("Floor: ")
    if (r>=200) and (g>=200) and (b>=200):
        print("White")
        robot.color_case ="white"
    elif (r>=230) and (g>=200) and (g<=240) and (b>110) and (b<=160):
        print("Orange")
        robot.color_case="orange"
    elif (r<70) and (g<70) and (b<70):
        print("Black")
        robot.color_case="black"

def sign_detection(img):
    #img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur = cv2.blur(gray,(1,1))
    blur_origin = cv2.blur(img,(1,1))
    hsv = cv2.cvtColor(blur_origin, cv2.COLOR_RGB2HSV)
    cv2.imshow("HSV" , hsv)
  
    #binary thresholding
    ret, thresh = cv2.threshold(blur, 135, 255, cv2.THRESH_BINARY) #135
    cv2.imshow("blur" , blur)

    #finding contours (white frame) in thresholded image
    cnts,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sign_pixel_count = cv2.countNonZero(thresh)

    print("number of contours found" , len(cnts))
    if(len(cnts) > 0):
    #cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cv2.imshow("thresh",thresh)
        
        
        #get co-ordinates of the contour
        sign = thresh[0:1 , 0:1]
        sign_colored =hsv[0:1 , 0:1]
        
        image_detected = False
        for c in cnts:
            [x,y,w,h] = cv2.boundingRect(cnts[0])
            if x in range(3, 33):
                #crop the image to be contour co-ordinates
                sign=thresh[y:y+w , x:x+w] #crop thresh img
                sign_colored=img[y:y+w , x:x+w] #crop blurred original img
                cv2.imshow("sign" , sign)
                cv2.imshow("sign colored" , sign_colored)
                image_detected = True
                break
        return sign , sign_colored , image_detected
    
    return None, None, False

 
def detect_hazards(sign_colored):

    sign_colored = cv2.cvtColor(sign_colored, cv2.COLOR_BGR2RGB)
    sign_type= "N"
    h, w,_= sign_colored.shape
    half= h//2
    bottom = sign_colored[half: , :]
    # #use the bottom layer
    # bottom = sign[quart*2:, :]
    # bottom_colored = sign_colored[quart*2:, :]
    # rgba(197,0,79,255)
    # print(sign_colored.tolist())

    #if len(sign_colored.tolist()) > 1:
    #    shaped = sign_colored.reshape(int(sign_colored.size/3), 3)
    #    if [197, 0, 79] in shaped.tolist():
    #        print('---------------------------------------')
    #        print('red')

    #mask orange color
    lower_orange = np.array([192,142,0]) 
    upper_orange= np.array([204,190,20])

    orange_mask = cv2.inRange(bottom ,lower_orange, upper_orange)
    #cv2.imshow("orange mask" , orange_mask)

    #check if the orange color exists in the img
    pixels = cv2.countNonZero(orange_mask)
    print(f"orange pixels= {pixels}")

    if pixels > 10:
        print("Organic Peroxide _ orange")
        sign_type="O"
        return 

    #mask red color
    lower_red = np.array ([197,0,98])
    upper_red= np.array([198,100,118])

    red_mask = cv2.inRange(bottom ,lower_red,upper_red)
    cv2.imshow("red mask" , red_mask)

    #check if the red color exists in the img
    pixels = cv2.countNonZero(red_mask)
    print(f"red pixels= {pixels}")

    if pixels > 10:
        print("Flammable Gas _ red")
        sign_type="F"
        return

    return sign_type


def letter_detection(sign):

    #inverse the img to color the letter(if the img is human) in white to be able to find contours in it
    letter= cv2.bitwise_not(sign)
    cv2.imshow("letter" , letter)

    #filling the background of the img in black
    h,w=letter.shape
    for x in range(0,h):
        for y in range (0,w):
            pixel = letter[x,y]

            if pixel<20:
                break
            else:
                letter[x,y] = (0)

        for y_inversed in range(w-1,0, -1):
            pixel = letter[x,y_inversed]
            if pixel <20:
                break
            else:
                letter[x,y_inversed] = (0)

    #find contours in the letter img
    cnts = cv2.findContours(letter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        (x,y,w,h) = cv2.boundingRect(c)
        #crop the img to the letter itself
        letter=letter[y:y+h, x:x+w]

    #letter detection
    h, w= letter.shape
    letter_type="N"

    third = h//3
    top = letter[:third, :]
    middle = letter[third:third*2, :]
    bottom=letter[third*2:, :]

    #finding contours in the three part of the image to be able to recognize the letter
    cnts = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c1=(len(cnts))

    cnts = cv2.findContours(middle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c2=(len(cnts))

    cnts = cv2.findContours(bottom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c3=(len(cnts))

    #check wheather a letter is detected and change the bool value if yes 
    print ("lETTER CODE: ", c1,c2,c3)
    if c1==1 and c3==1:
        print("S victim")
        letter_type="S"
    elif c1==2 and c2==1 and c3==2:
        print("H victim")
        letter_type="H"
    
    elif c1==2 and c2==2 and c3==1:
        print("U victim")
        letter_type="U"

    return letter_type, bottom


def hazard_sign_detection(bottom):

    #make the background in gray
    h,w=bottom.shape
    for x in range(0,h):
        for y in range (0,w):
            pixel = bottom[x,y]
            if pixel>200:
                break
            else:
                bottom[x,y] = (120)

        for y_inversed in range(w-1,0, -1):
            pixel = bottom[x,y_inversed]
        
            if pixel >200:
                break
            else:
                bottom[x,y_inversed] = (120)
     
    #count the white and black pixels
    white_pixel=0
    black_pixel=0

    for x in range(0,h):
        for y in range (0,w):
            pixel = bottom[x,y]
            if pixel>200:
                white_pixel+=1
            elif pixel<20:
                black_pixel+=1

    #compare between the two numbers
    print(f"white pixels: {white_pixel} black pixels: {black_pixel}")

    if(black_pixel>white_pixel):
        print("Corrosive_Black")
        sign_type="C"
    else:
        print("POISON_white")
        sign_type="P"
    
        
def cam(robot: MazeRobot , img):

    sign , sign_colored , image_detected= sign_detection(img)
    if image_detected:
        sign_type = detect_hazards(sign_colored)
        if(sign_type=="N"):
            letter_type, bottom = letter_detection(sign)
            if letter_type =="N":
                hazard_sign_detection(bottom)
                    
        cv2.waitKey(1)



    """
    gray= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur =  cv2.blur(gray,(3,3))
    cv2.imshow("gray" , gray)
    cv2.imshow("blur" , blur)

    ret, thresh1 = cv2.threshold(blur, 170, 255, cv2.THRESH_BINARY)
    #cv2.imshow("Blur",blur)
    #cv2.imshow('Binary Threshold', thresh1)
    thresh2=thresh1.copy()

    mask = cv2.inRange(thresh1,(180), (255)) # masking white

    #cv2.imshow("maskkk",mask)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    maskcopy=mask.copy()
    cv2.fillPoly(maskcopy, cnts, (255,255,255))

    thresh2[maskcopy>0]=(255)
    #cv2.imshow("maskcopy",maskcopy)

    mask2 = cv2.inRange(thresh2,(0), (50))
    thresh1[mask2>0]=(255)

    thresh1 = cv2.bitwise_not(thresh1)
    cv2.imshow("new" ,thresh1)


    cnts = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    final=thresh1.copy()
    for c in cnts:
        (x,y,w,h) = cv2.boundingRect(c)
        final=thresh1[y:y+h, x:x+w]
        #cv2.rectangle(final,(x,y),(x+w,y+h),(255,0,0),5)
        cv2.imshow("final" , final)

    h, w= final.shape
    # this is horizontal division
    third = h//3

    top = final[:third, :]
    middle = final[third:third*2, :]
    bottom=final[third*2:, :]

    #cv2.imshow('Top', top)
    cnts = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c1=(len(cnts))

    cnts = cv2.findContours(middle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c2=(len(cnts))

    cnts = cv2.findContours(bottom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c3=(len(cnts))

    if c1==1 and c2==1 and c3==1:
        print("S victim")
    elif c1==2 and c2==1 and c3==2:
        print("H victim")
    elif c1==2 and c2==2 and c3==1:
        print("U victim")
    #cv2.imshow('Middle', middle)
    #cv2.imshow('Bottom', bottom)
    cv2.waitKey(1)
    """


    """
    coords_list = []
    img = np.frombuffer(img2, np.uint8).reshape((robot.right_camera.getHeight(), robot.right_camera.getWidth(), 4))


    #convert from BGR to HSV color space
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #apply threshold
    #blur = cv2.blur(gray,(5,5))
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]



    # draw all contours in green and accepted ones in red
    contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    for c in contours:
        if cv2.contourArea(c) > 1000:
            x,y,w,h = cv2.boundingRect(c)
            ROI = image[y:y+h, x:x+w]
            coords = list(c[0][0])
            coords_list.append(coords)
            print("VICTIIM")
            #print("Victim at x="+str(coords[0])+" y="+str(coords[1]))
            my_vision = RCJRVision.HSUVision()
            letter, center = my_vision.find_HSU(img)
            letter = str(letter)
            if(letter=='S' or letter=='H' or letter=='U'):
                robot.victim_status=letter
                print(letter)
                if robot.victim_timer!=0: robot.victim_timer=robot.robot.getTime()
                if (robot.robot.getTime() - robot.victim_timer) < 3.0:
                    stop(robot)
                else: robot.victim_timer=0

    
   


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
    
"""


