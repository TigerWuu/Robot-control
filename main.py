import pyrealsense2 as rs
import numpy as np
import cv2
import serial
from time import sleep
from timeit import default_timer as timer
from PIL import Image, ImageFont, ImageDraw
import sys
from math import *
import Robot
# Configure depth and color streams
Rc=Robot.Robot_control()

COM_PORT = 'COM7'  
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES)

camera_length = 640
camera_width = 480

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, camera_length , camera_width , rs.format.z16, 30)
config.enable_stream(rs.stream.color, camera_length , camera_width , rs.format.bgr8, 30)

accum_time = 0
curr_fps = 0
fps = "FPS: ??"
prev_time = timer()

# Start streaming
pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

camera_factor = 1
camera_cx = 321.798
camera_cy = 239.607
camera_fx = 615.899
camera_fy = 616.468

#pixel to xy coordinate
def coord_transfer(x,y,z):
    cz = z / camera_factor
    cx = (x - camera_cx) * cz / camera_fx
    cy = (y - camera_cy) * cz / camera_fy
    return cx , cy , cz

def rad_sort(a):
    for i in range(len(a)):
        for j in range(len(a)-1):
            if a[j+1][2]>a[j][2]:
                temp=a[j]
                a[j]=a[j+1]
                a[j+1]=temp
    return a

circle_size = 0
while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth = np.asanyarray(aligned_depth_frame.get_data())
    color = np.asanyarray(color_frame.get_data())
    # Convert numpy arrays to images
    depth_image = Image.fromarray(depth)
    color_image = Image.fromarray(color)

    #find circle
    img_gray=cv2.cvtColor(color , cv2.COLOR_BGR2GRAY)
    img_blur=cv2.blur(img_gray,(3,3))
    ret, img_thres=cv2.threshold(img_blur,100,255,cv2.THRESH_BINARY)
    img_edge = cv2.Canny(img_thres,50,150)
    circles=cv2.HoughCircles(img_edge,cv2.HOUGH_GRADIENT,1,100,param1=80,param2=35,minRadius=10,maxRadius=100)  # check
    
    #print(len(circles[0]))
    coordinate=[]
    if circles is not None and circles[0][0].ndim == 1:      ##check
        circles = np.int0(np.round(circles))
        c_x = np.zeros(len(circles[0]))
        c_y = np.zeros(len(circles[0]))
        c_z = np.zeros(len(circles[0]))
        coordinate = np.ones((len(circles[0]),4))

        circles[0] = rad_sort(circles[0])
        times = 0
        for i in circles[0]:
            x = int(i[0])
            y = int(i[1])
            r = int(i[2])

            cv2.circle(color,(x,y),r,(0,0,255),2)
            cv2.circle(color,(x,y),3,(0,255,0),-1)
            if y<480 and x+r+20<640:
                z =depth[y,x+r+20]
            else:
                continue
            #pixel_coordinate to camera_coordinate (org in the frame center)
            coordinate[times][0] , coordinate[times][1] , coordinate[times][2] = coord_transfer(x,y,z)
            center_coord = "( %.1f , %.1f , %.1f )" % (coordinate[times][0] , coordinate[times][1] , coordinate[times][2])
            #show the coordinate beside the center of circles
            cv2.putText(color, text=center_coord, org=(x+5, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.40, color=(0, 255, 255), thickness=2)
            times = times+1

    #print(coordinate)
    #k = gain
    """
    目標物在車體右邊
    k = 0.30       #gain  
    if 找到三個圓:
        將圓從大排到小
        if (相機中心-圓中心) < 30mm:
            velocity = 0
        else :
            velocity = (相機中心-圓中心) * k

        if velocity > 0:
            go backward
        else:
            go forward
    else:
        跑很快
    """
    # 1 forward , -1 backward , 0 stop
    
    if np.size(coordinate,0) == 3:

        if coordinate[circle_size][0] < 0 and -coordinate[circle_size][0] > 30:
            velocity = "35,1,0,0,-1\n"
            ser.write(velocity.encode())

        elif coordinate[circle_size][0] > 0 and coordinate[circle_size][0] > 30:
            velocity = "35,-1,0,0,-1\n"
            ser.write(velocity.encode())
    
        else:
            velocity = "0,0,0,0,-1\n"
            for i in range(500):               
                ser.write(velocity.encode())

            #calculate the forward kinematic and inverse
            #TRANS = Rc.forward_kinematics(pi/2 , 7*pi/9 , 4*pi/9 , 0)
            TRANS = Rc.forward_kinematics(pi/2 , 7*pi/9 , 4*pi/9 , 11*pi/18) 
            coord = np.reshape(coordinate[circle_size], (4, 1))

            #T=np.array([[1,0,0,0],[0,cos(-5*pi/18),-sin(-5*pi/18),0],[0,sin(-5*pi/18),cos(-5*pi/18),230],[0,0,0,1]])
            arm_coord = TRANS.dot(Rc.rotation_matrix("y",-pi/2)).dot(Rc.rotation_matrix("x",pi)).dot(coord)
 
            print(arm_coord)

            thetas = Rc.inverse_kinematics(arm_coord[0][0],arm_coord[1][0],arm_coord[2][0], -pi/4)
            print("shit:",thetas)

            if thetas != 0:
                print("thetas is available!!")
                for i in range(500):  
                    ser.write(thetas.encode())

                while True:
                    #waiting for the arduino to transport the message
                    if ser.in_waiting :
                        print("ser:",ser.in_waiting)
                        data_raw = ser.readline() 
                        data = data_raw.decode()
                        print("before : ",data_raw) 
                        print("after : ",data)

                        if data == "done\r\n" :
                            print("circle_size :",circle_size)
                            if circle_size < 2:
                                circle_size = circle_size + 1
                            break
            else:
                print("thetas is not available!!")
                ttt = str(500) + "," + str(500) + "," + str(500) + "," + str(500) + "," + str(500) +  "\n"
                for i in range(500):  
                    ser.write(ttt.encode())
                
                while True:
                    #waiting for the arduino to transport the message
                    if ser.in_waiting :
                        print("ser:",ser.in_waiting)
                        data_raw = ser.readline() 
                        data = data_raw.decode()
                        print("before : ",data_raw) 
                        print("after : ",data)

                        if data == "done\r\n" :
                            print("circle_size :",circle_size)
                            if circle_size < 2:
                                circle_size = circle_size + 1
                            break

    else:
        ser.write(("35,1,0,0,-1\n").encode())

    curr_time = timer()
    exec_time = curr_time - prev_time
    prev_time = curr_time
    accum_time = accum_time + exec_time
    curr_fps = curr_fps + 1
    if accum_time > 1:
        accum_time = accum_time - 1
        fps = "FPS: " + str(curr_fps)
        curr_fps = 0

    cv2.circle(color,(50,50),4,(0,0,255),-1)
    cv2.putText(color, text=fps, org=(3, 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.50, color=(255, 0, 0), thickness=2)
    #cv2.namedWindow("result", cv2.WINDOW_NORMAL)
    cv2.imshow("realsense_b&w", img_edge)
    cv2.imshow("realsense_color", color)
    
    #cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("finished")
        break



