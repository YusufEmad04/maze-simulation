from MazeRobot import MazeRobot
import cv2
import numpy as np

counter = 0

def get_mean_color(img):
    # get the mean color of the image
    mean_color = np.mean(img, axis=(0, 1))
    return mean_color

def sign_is_mostly_blue(img):
    '''
    b 125, 150
    g 110, 130
    r 20 70
    '''

    lower = np.array([120, 100, 20])
    upper = np.array([160, 145, 80])

    mean_color = get_mean_color(img)

    if np.all(mean_color >= lower) and np.all(mean_color <= upper) and mean_color[0] > mean_color[1] > mean_color[2]:
        return True

def contour_without_blue(img):
    image_dimensions = img.shape

    upper_left = img[0:int(image_dimensions[0]/2), 0:int(image_dimensions[1]/2)]
    upper_right = img[0:int(image_dimensions[0]/2), int(image_dimensions[1]/2):image_dimensions[1]]
    lower_left = img[int(image_dimensions[0]/2):image_dimensions[0], 0:int(image_dimensions[1]/2)]
    lower_right = img[int(image_dimensions[0]/2):image_dimensions[0], int(image_dimensions[1]/2):image_dimensions[1]]

    quadrants = [upper_left, upper_right, lower_left, lower_right]

    for quadrant in quadrants:
        if sign_is_mostly_blue(quadrant):
            return False

    return True

def get_image_contours(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0, 0, 110]) # lower white
    upper1 = np.array([180, 40, 255]) # upper white

    mask1 = cv2.inRange(hsv_image, lower1, upper1) # mask white

    lower2 = np.array([0, 50, 50]) # lower1 red
    upper2 = np.array([50, 255, 255]) # upper1 red

    mask2 = cv2.inRange(hsv_image, lower2, upper2) # mask red

    lower3 = np.array([125, 0, 0]) # lower2 red
    upper3 = np.array([180, 255, 255]) # upper2 red

    mask3 = cv2.inRange(hsv_image, lower3, upper3) # mask red

    mask_red = cv2.bitwise_or(mask3, mask2) # mask red

    mask = cv2.bitwise_or(mask1, mask_red) # mask white and red

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return cnts, mask

def sign_detection(img):
    global counter
    counter += 1

    cnts, binary_image = get_image_contours(img)

    # print("number of contours found" , len(cnts))
    if len(cnts) > 0:
        # cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        # cv2.imshow("thresh",thresh)

        # get co-ordinates of the contour
        sign = None
        sign_colored = None

        image_detected = False
        for c in cnts:
            [x, y, w, h] = cv2.boundingRect(c)

            sign_colored = img[y:y + h, x:x + w]

            if w > 18 and h > 18 and (5 < x < 20) and y < 25 and contour_without_blue(sign_colored):

                img_copy = img.copy()
                # draw bounding box
                cv2.rectangle(img_copy, (x, y), (x + w, y + h), (36, 255, 12), 2)

                cv2.imwrite("images/" + str(counter) + ".png", img_copy)
                counter += 1
                # save the image
                # crop the image to be contour co-ordinates
                sign = binary_image[y:y + h, x:x + w]  # crop thresh img
                  # crop blurred original img
                # cv2.imshow("sign" , sign)
                # cv2.imshow("sign colored" , sign_colored)
                image_detected = True
                break
        return sign, sign_colored, image_detected

    return None, None, False


def detect_hazards(sign_colored):
    sign_colored = cv2.cvtColor(sign_colored, cv2.COLOR_BGR2RGB)
    sign_type = "N"
    h, w, _ = sign_colored.shape
    half = h // 2
    bottom = sign_colored[half:, :]
    # #use the bottom layer
    # bottom = sign[quart*2:, :]
    # bottom_colored = sign_colored[quart*2:, :]
    # rgba(197,0,79,255)
    # print(sign_colored.tolist())

    # if len(sign_colored.tolist()) > 1:
    #    shaped = sign_colored.reshape(int(sign_colored.size/3), 3)
    #    if [197, 0, 79] in shaped.tolist():
    #        print('---------------------------------------')
    #        print('red')

    # mask orange color
    lower_orange = np.array([192, 142, 0])
    upper_orange = np.array([204, 190, 20])

    orange_mask = cv2.inRange(bottom, lower_orange, upper_orange)
    # cv2.imshow("orange mask" , orange_mask)

    # check if the orange color exists in the img
    pixels = cv2.countNonZero(orange_mask)
    # print(f"orange pixels= {pixels}")
    # TODO Organic Peroxide sign
    if pixels > 10:
        # print("Organic Peroxide _ orange")
        sign_type = "O"
        # return

    # mask red color
    lower_red = np.array([185, 0, 0])
    upper_red = np.array([205, 100, 118])

    red_mask = cv2.inRange(bottom, lower_red, upper_red)
    # cv2.imshow("red mask" , red_mask)

    # check if the red color exists in the img
    pixels = cv2.countNonZero(red_mask)
    # print(f"red pixels= {pixels}")
    #
    # # save red mask
    # cv2.imwrite("images/red_mask{}.png".format(counter), red_mask)
    #
    # # save bottom
    # cv2.imwrite("images/bottom{}.png".format(counter), bottom)

    if pixels > 25:
        # print("Flammable Gas _ red")
        sign_type = "F"
        # return

    return sign_type


def letter_detection(sign):
    # inverse the img to color the letter(if the img is human) in white to be able to find contours in it
    letter = cv2.bitwise_not(sign)
    # cv2.imshow("letter" , letter)
    # save letter

    # filling the background of the img with black
    h, w = letter.shape
    for x in range(0, h):
        for y in range(0, w):
            pixel = letter[x, y]

            if pixel < 20:
                break
            else:
                letter[x, y] = (0)

        for y_inversed in range(w - 1, 0, -1):
            pixel = letter[x, y_inversed]
            if pixel < 20:
                break
            else:
                letter[x, y_inversed] = (0)

    # find contours in the letter img
    cnts = cv2.findContours(letter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        # crop the img to the letter itself
        letter = letter[y:y + h, x:x + w]

    # letter detection
    h, w = letter.shape
    letter_type = "N"

    third = h // 3
    top = letter[:third, :]
    middle = letter[third:third * 2, :]
    bottom = letter[third * 2:, :]

    # finding contours in the three part of the image to be able to recognize the letter
    cnts = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c1 = (len(cnts))

    cnts = cv2.findContours(middle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c2 = (len(cnts))

    cnts = cv2.findContours(bottom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c3 = (len(cnts))

    # check whether a letter is detected and change the bool value if yes
    # print ("LETTER CODE: ", c1,c2,c3)
    if c1 == 1 and c3 == 1:
        # print("S victim")
        letter_type = "S"
    elif c1 == 2 and c2 == 1 and c3 == 2:
        # print("H victim")
        letter_type = "H"

    elif c1 == 2 and c2 == 2 and c3 == 1:
        # print("U victim")
        letter_type = "U"

    return letter_type, bottom


def hazard_sign_detection(sign):
    # make the background in gray
    sign_type = "N"
    h, w = sign.shape
    # get bottom half of the image
    bottom = sign[int(h * 0.5):, int(w * 0.3):int(w * 0.85)]
    # save bottom

    # for x in range(0,h):
    #     for y in range (0,w):
    #         pixel = bottom[x,y]
    #         if pixel>200:
    #             break
    #         else:
    #             bottom[x,y] = (120)
    #
    #     for y_inversed in range(w-1,0, -1):
    #         pixel = bottom[x,y_inversed]
    #
    #         if pixel >200:
    #             break
    #         else:
    #             bottom[x,y_inversed] = (120)
    #
    # #count the white and black pixels
    # white_pixel=0
    # black_pixel=0
    #
    # for x in range(0,h):
    #     for y in range (0,w):
    #         pixel = bottom[x,y]
    #         if pixel>200:
    #             white_pixel+=1
    #         elif pixel<20:
    #             black_pixel+=1

    white_pixel = cv2.countNonZero(bottom)
    black_pixel = bottom.size - white_pixel
    # compare between the two numbers
    print(f"white pixels: {white_pixel} black pixels: {black_pixel}")

    if black_pixel > white_pixel:
        # print("Corrosive_Black")
        sign_type = "C"
    else:
        # print("POISON_white")
        sign_type = "P"
    return sign_type


def moving_cam(img):
    sign, sign_colored, image_detected = sign_detection(img)
    return image_detected


def full_detection(img):
    sign, sign_colored, image_detected = sign_detection(img)
    # if image_detected:
    sign_type = detect_hazards(sign_colored)
    if sign_type != "N":
        return sign_type
    letter_type, bottom = letter_detection(sign)
    if letter_type != "N":
        return letter_type
    sign_type = hazard_sign_detection(sign)
    return sign_type
