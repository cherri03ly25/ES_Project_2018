#! /usr/bin/env python3

from car_control import car_controller
from wifi_connect import wifi_connector
from mvnc import mvncapi as mvnc
from sys import argv
import sys
import numpy
import cv2
import time
import csv
import os

# name of the opencv window
cv_window_name = "vehicle platooning"

# labels AKA classes.  The class IDs returned
# are the indices into this list
labels = ('background',
          'aeroplane', 'bicycle', 'bird', 'boat',
          'bottle', 'bus', 'car', 'cat', 'chair',
          'cow', 'diningtable', 'dog', 'horse',
          'motorbike', 'person', 'pottedplant',
          'sheep', 'sofa', 'train', 'tvmonitor')

# the ssd mobilenet image width and height
NETWORK_IMAGE_WIDTH = 300
NETWORK_IMAGE_HEIGHT = 300

# the minimal score for a box to be shown
min_score_percent = 60

# the resize_window arg will modify these if its specified on the commandline
resize_output = False
resize_output_width = 0
resize_output_height = 0

# read video files from this directory
input_video_path = '.'


def preprocess_image(source_image):
    resized_image = cv2.resize(source_image, (NETWORK_IMAGE_WIDTH, NETWORK_IMAGE_HEIGHT))

    # trasnform values from range 0-255 to range -1.0 - 1.0
    resized_image = resized_image - 127.5
    resized_image = resized_image * 0.007843
    return resized_image


def handle_keys(raw_key):
    global min_score_percent
    ascii_code = raw_key & 0xFF
    if ((ascii_code == ord('q')) or (ascii_code == ord('Q'))):
        return False
    elif (ascii_code == ord('B')):
        min_score_percent += 5
        print('New minimum box percentage: ' + str(min_score_percent) + '%')
    elif (ascii_code == ord('b')):
        min_score_percent -= 5
        print('New minimum box percentage: ' + str(min_score_percent) + '%')

    return True

def overlay_on_image(display_image, object_info):
    source_image_width = display_image.shape[1]
    source_image_height = display_image.shape[0]

    base_index = 0
    class_id = object_info[base_index + 1]
    percentage = int(object_info[base_index + 2] * 100)
    if (percentage <= min_score_percent):
        return

    label_text = labels[int(class_id)] + " (" + str(percentage) + "%)"
    box_left = int(object_info[base_index + 3] * source_image_width)
    box_top = int(object_info[base_index + 4] * source_image_height)
    box_right = int(object_info[base_index + 5] * source_image_width)
    box_bottom = int(object_info[base_index + 6] * source_image_height)

    box_color = (255, 128, 0)  # box color
    box_thickness = 2
    cv2.rectangle(display_image, (box_left, box_top), (box_right, box_bottom), box_color, box_thickness)

    scale_max = (100.0 - min_score_percent)
    scaled_prob = (percentage - min_score_percent)
    scale = scaled_prob / scale_max

    # draw the classification label string just above and to the left of the rectangle
    #label_background_color = (70, 120, 70)  # greyish green background for text
    label_background_color = (0, int(scale * 175), 75)
    label_text_color = (255, 255, 255)  # white text

    label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    label_left = box_left
    label_top = box_top - label_size[1]
    if (label_top < 1):
        label_top = 1
    label_right = label_left + label_size[0]
    label_bottom = label_top + label_size[1]
    cv2.rectangle(display_image, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1),
                  label_background_color, -1)

    # label text above the box
    cv2.putText(display_image, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_text_color, 1)

    # display text to let user know how to quit
    cv2.rectangle(display_image,(0, 0),(100, 15), (128, 128, 128), -1)
    cv2.putText(display_image, "Q to Quit", (10, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)


def handle_args():
    global resize_output, resize_output_width, resize_output_height
    for an_arg in argv:
        if (an_arg == argv[0]):
            continue

        elif (str(an_arg).lower() == 'help'):
            return False

        elif (str(an_arg).startswith('resize_window=')):
            try:
                arg, val = str(an_arg).split('=', 1)
                width_height = str(val).split('x', 1)
                resize_output_width = int(width_height[0])
                resize_output_height = int(width_height[1])
                resize_output = True
                print ('GUI window resize now on: \n  width = ' +
                       str(resize_output_width) +
                       '\n  height = ' + str(resize_output_height))
            except:
                print('Error with resize_window argument: "' + an_arg + '"')
                return False
        else:
            return False

    return True

def chessboard(img):
	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = numpy.zeros((6*7,3), numpy.float32)
	objp[:,:2] = numpy.mgrid[0:7,0:6].T.reshape(-1,2)

	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.

	#images = glob.glob('*.png')


	#img = cv2.imread('template.png')
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (5,5),None)

	# If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)

		#corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		#imgpoints.append(corners2)

		# Draw and display the corners
		img = cv2.drawChessboardCorners(img, (7,7), corners,ret)
		#cv2.imshow('img',img)
		#cv2.waitKey(500)

	#cv2.destroyAllWindows()

def run_camera_calibration(cap, nrows, ncols, dimension):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((nrows * ncols, 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:ncols, 0:nrows].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    ii = 0
    while True:
        ret, img = cap.read()
        if (not ret):
            end_time = time.time()
            print("No image from from video device, exiting")
            break

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (nrows, ncols), None)
        cv2.imshow(cv_window_name, gray)
        if ret == True:
            ii +=1
            print("cornners found :", ii)
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (nrows, ncols), corners2, ret)
            # cv2.imshow('chessboard corners', img)
            cv2.imshow(cv_window_name, img)
            cv2.waitKey(500)
        else:
            print("cornners not found")
            # TODO: when we can controll the car, we can do this automatically by adjust the distance to the front vehicle
        choice = input('Collect more object points, enter  N to stop, press enter to continue:')
        if choice.upper() == 'N':
            break;

    cv2.destroyAllWindows()

    #return ret, camera matrix, distortion coefficients, rotation and translation vectors
    return  cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

	
def run_inference(img, graphnet):

    # preprocess the image to meet nework expectations
    resized_image = preprocess_image(img)

    graphnet.LoadTensor(resized_image.astype(numpy.float16), None)

    output, userobj = graphnet.GetResult()

    #   a.	First fp16 value holds the number of valid detections = num_valid.
    #   b.	The next 6 values are unused.
    #   c.	The next (7 * num_valid) values contain the valid detections data
    #       Each group of 7 values will describe an object/box These 7 values in order.
    #       The values are:
    #         0: image_id (always 0)
    #         1: class_id (this is an index into labels)
    #         2: score (this is the probability for the class)
    #         3: box left location within image as number between 0.0 and 1.0
    #         4: box top location within image as number between 0.0 and 1.0
    #         5: box right location within image as number between 0.0 and 1.0
    #         6: box bottom location within image as number between 0.0 and 1.0

    # number of boxes returned
    num_valid_boxes = int(output[0])

    for box_index in range(num_valid_boxes):
            base_index = 7+ box_index * 7
            if (not numpy.isfinite(output[base_index]) or
                    not numpy.isfinite(output[base_index + 1]) or
                    not numpy.isfinite(output[base_index + 2]) or
                    not numpy.isfinite(output[base_index + 3]) or
                    not numpy.isfinite(output[base_index + 4]) or
                    not numpy.isfinite(output[base_index + 5]) or
                    not numpy.isfinite(output[base_index + 6])):
                # boxes with non finite (inf, nan, etc) numbers must be ignored
                continue

            x1 = max(int(output[base_index + 3] * img.shape[0]), 0)
            y1 = max(int(output[base_index + 4] * img.shape[1]), 0)
            x2 = min(int(output[base_index + 5] * img.shape[0]), img.shape[0]-1)
            y2 = min((output[base_index + 6] * img.shape[1]), img.shape[1]-1)

            # overlay boxes and labels on to the image
            overlay_on_image(img, output[base_index:base_index + 7])


# prints usage information
def print_usage():
    print('\nusage: ')
    print('python3 run_video.py [help][resize_window=<width>x<height>]')
    print('')
    print('options:')
    print('  help - prints this message')
    print('  resize_window - resizes the GUI window to specified dimensions')
    print('                  must be formated similar to resize_window=1280x720')
    print('')
    print('Example: ')
    print('python3 run_video.py resize_window=1920x1080')


def get_dx(corners):
    return abs(tuple(corners[3].ravel())[0] - tuple(corners[0].ravel())[0])

def get_dy(corners):
    return abs(tuple(corners[12].ravel())[0] - tuple(corners[0].ravel())[0])

def get_area(corners):
    o = corners[0].ravel()
    epx = corners[3].ravel()
    epy = corners[12].ravel()
    return cv2.norm(epx - o)* cv2.norm(epy - o)

def get_y(img, corners, imgpts):
    corner = tuple(corners[4].ravel())
    corners[4].ravel()

    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def set_speed(Area, Ai, gain):
    run = 0
    speed = 0
    if Area < Ai:
        run = 1
        speed = (Ai-Area)*gain
    return run, int(speed)

def set_steering(dx, gain):
    return int(dx*gain)

def main():
    global resize_output, resize_output_width, resize_output_height

    if (not handle_args()):
        print_usage()
        return 1

    mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)


    devices = mvnc.EnumerateDevices()
    if len(devices) == 0:
        print('No devices found')
        quit()

    device = mvnc.Device(devices[0])
    device.OpenDevice()

    graph_filename = 'graph'
    with open(graph_filename, mode='rb') as f:
        graph_data = f.read()


    graphnet = device.AllocateGraph(graph_data)



    # template = cv2.imread('template.png',0)
    # template = cv2.resize(template, (NETWORK_IMAGE_WIDTH, NETWORK_IMAGE_HEIGHT))
    # orb = cv2.ORB_create()
    # kp1, des1 = orb.detectAndCompute(template,None)
    cv2.namedWindow(cv_window_name)
    cv2.moveWindow(cv_window_name, 10,  10)

    exit_app = False

    cap = cv2.VideoCapture(0)

    actual_frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print ('actual video resolution: ' + str(actual_frame_width) + ' x ' + str(actual_frame_height))

    if ((cap == None) or (not cap.isOpened())):
        print ('Could not open video device. ')
        exit_app = True


    frame_count = 0
    start_time = time.time()
    end_time = start_time

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    ##  Calibration Matrix:
    ##[[ 516.14758188    0.          314.02546443]
    ## [   0.          515.76615942  250.15817809]
    ## [   0.            0.            1.        ]]
    ##Disortion:  [[  2.48041485e-01  -6.31759025e-01   4.36060601e-04  -1.48720850e-03
    ##    5.17810257e-01]]
    ##total error:  0.021112667972552
    mtx = numpy.matrix([[516.14758188, 0 , 314.02546443], [0 , 515.76615942 , 250.15817809], [0, 0, 1]])
    disto = numpy.matrix([[2.48041485e-01,  -6.31759025e-01 ,  4.36060601e-04, -1.48720850e-03, 5.17810257e-01]])

    #TODO:  configurable values
    nrows = 4#7
    ncols = 4#7
    dimension = 18#9
    Qi = 20
    Ai = 8980
    Gspeed = 1
    Gsteering = 1
    tpx = int(actual_frame_width / 2)
    tpy = int(actual_frame_height / 2) + int(actual_frame_height / 8)

    car1 = car_controller(1)
    
    debug = False
    # xtarget =
    if debug == True:
        choice = input('Enter Y to run camera calibration, press enter to continue:')
        if choice.upper() == 'Y':
            ret, mtx, disto, rvecs, tvecs = run_camera_calibration(cap,nrows, ncols, dimension)
            if not ret:
                print('failed to calibrate')
                exit_app = True

    print('mtx',mtx)
    print('disto', disto)
    ret, img = cap.read()
    h, w = img.shape[:-1]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, disto, (w, h), 1, (w, h))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((nrows * ncols, 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:nrows, 0:ncols].T.reshape(-1, 2)
    axis = numpy.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
    cmd_queue = []
    cmd_queue_size = 0
    while(True):
        if (exit_app):
            break
        ret, image = cap.read()
        if (not ret):
            end_time = time.time()
            print("No image from from video device, exiting")
            break
        #TODO: undistort image
        # image_ud = cv2.undistort(image, mtx, disto, None, newcameramtx)
        # x, y, w, h = roi
        # image_ud = image_ud[y:y + h, x:x + w]

        # check if user hasn't closed the window
        prop_val = cv2.getWindowProperty(cv_window_name, cv2.WND_PROP_ASPECT_RATIO)
        if (prop_val < 0.0):
            end_time = time.time()
            exit_app = True
            break
        ###################################################################################################
        # kp2, des2 = orb.detectAndCompute(image_ud,None)
        # matches = bf.match(des1,des2)
        # matches = sorted(matches, key = lambda x:x.distance)
        # display_image = cv2.drawMatches(template,kp1,image_ud,kp2,matches[:36],None, flags=4)
        ####################################################################################################
        # run_inference(display_image, graphnet)
        # if (resize_output):
        # display_image = cv2.resize(display_image,(resize_output_width, resize_output_height), cv2.INTER_LINEAR)
        ####################################################################################################
##################################################################################################
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret = True
        #corners = cv2.cornerHarris(gray,2,3,0.04)
        #corners = cv2.goodFeaturesToTrack(gray,16, 0.1, 10)
        ret, corners = cv2.findChessboardCorners(gray, (nrows, ncols), None)
        print('ret :', ret)
        print(corners)
        if ret != True:
            continue
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # Find the rotation and translation vectors.
        _ , rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, disto)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, disto)
        gray= draw(gray, corners2, imgpts)

        for corner in corners:
            [x, y] = corner[0]
            cv2.circle(gray, (x, y), 3, 255, -1)

        o1 = corners[5].ravel()
        e1 = corners[10].ravel()
        o2 = corners[6].ravel()
        e2 = corners[9].ravel()

        x = o2 - o1;
        d1 = e1 - o1;
        d2 = e2 - o2;
        xx = tuple(x)[0]
        xy = tuple(x)[1]
        d1x = tuple(d1)[0]
        d1y = tuple(d1)[1]
        d2x = tuple(d2)[0]
        d2y = tuple(d2)[1]

        cross = d1x * d2y - d1y * d2x;
        if (abs(cross) < 1e-8) :
            print("Error")

        t1 = (xx * d2y - xy * d2x) / cross;
        center = o1 + d1 * t1;
        cmd_steering_dx = tuple(center)[0] - tpx
        area = get_area(corners2)
        run, m = set_speed(area, Ai, Gspeed)
        s = set_steering(cmd_steering_dx,Gsteering)

        print('cmd_steering_dx :', cmd_steering_dx)
        print('Area :', area)
        car1.write_to_arduino(run, s, m)

        #TODO: check the reason in area variation

        # if cmd_queue_size == 0 :
        #     n = int(area*(Qi/Ai))
        #     for ii in range(n):
        #         cmd_queue.append([tpx,actual_frame_height,ii])
        #         #cv2.line(gray, (tpx,actual_frame_height,ii), (tpx,actual_frame_height,ii+1), (0, 0, 255), 2)
        #         cv2.circle(gray, (tpx,actual_frame_height,ii), 1, [0, 0, 0], -1)



        cv2.line(gray, tuple(e1), tuple(o1), (255,255,255), 2)
        cv2.line(gray, tuple(e2), tuple(o2), (255, 255, 255), 2)
        cv2.circle(gray, tuple(center), int(xx/8), (255, 255, 255), -1)
        cv2.circle(gray, (tpx, tpy), 10, [150, 0, 0], -1)
        cv2.line(gray, tuple(center), (tpx, tuple(center)[1]), (255, 255, 255), 2)
        cv2.line(gray, (tpx, int(tuple(center)[1]) + int(xx/8)), (tpx, int(tuple(center)[1]) - int(xx/8)), (255, 255, 255), 2)
        if(cmd_steering_dx < 0) :
            cv2.putText(gray,"<-- %d" % cmd_steering_dx , (tpx, int(tuple(center)[1])), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(gray, "%d -->" % cmd_steering_dx, (tpx, int(tuple(center)[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(cv_window_name, gray)

        raw_key = cv2.waitKey(1)
        if (raw_key != -1):
            if (handle_keys(raw_key) == False):
                end_time = time.time()
                exit_app = True
                break
        frame_count += 1


    frames_per_second = frame_count / (end_time - start_time)
    print('Frames per Second: ' + str(frames_per_second))

    cap.release()

    # Clean up the graph and the device
    graphnet.DeallocateGraph()
    device.CloseDevice()


    cv2.destroyAllWindows()

if __name__ == "__main__":
    sys.exit(main())
