from MazeRobot import MazeRobot
import cv2
import numpy as np



def sign_detection(img):
    
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
    sign_type="N"
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
    return sign_type
    

def moving_cam(img):
    sign , sign_colored , image_detected= sign_detection(img)
    return image_detected


def full_detection(img):

    sign , sign_colored , image_detected= sign_detection(img)
    # if image_detected:
    sign_type = detect_hazards(sign_colored)
    if(sign_type!="N"):
        return sign_type
    letter_type, bottom = letter_detection(sign)
    if letter_type !="N":
        return letter_type
    sign_type = hazard_sign_detection(bottom)
    return sign_type

                
    cv2.waitKey(1)