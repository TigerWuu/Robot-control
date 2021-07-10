from math import *
import numpy as np


class Robot_control(object):
    def __init__(self):
        self.l1 = 23
        self.l2 = 100.4    
        self.l3 = 170
        self.l4 = 95
        self.realsense_l4 = 63


    def forward_kinematics(self, theta1, theta2, theta3, theta4):
        

        Trans=np.array([
        [ (81129638414606686663546605165575*cos(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128 + (81129638414606676728031405122553*cos(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128, - (81129638414606676728031405122553*sin(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128 - (81129638414606686663546605165575*sin(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128,                                       sin(theta1), (81129638414606676728031405122553*self.l3*cos(theta2 - theta1 + theta3))/162259276829213363391578010288128 + (81129638414606686663546605165575*self.realsense_l4*cos(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128 + (81129638414606686663546605165575*self.l2*cos(theta1 + theta2))/162259276829213363391578010288128 + (81129638414606676728031405122553*self.realsense_l4*cos(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128 + (81129638414606676728031405122553*self.l2*cos(theta1 - theta2))/162259276829213363391578010288128 + (81129638414606686663546605165575*self.l3*cos(theta1 + theta2 + theta3))/162259276829213363391578010288128],
        [ (81129638414606686663546605165575*sin(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128 - (81129638414606676728031405122553*sin(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128,   (81129638414606686663546605165575*cos(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128 - (81129638414606676728031405122553*cos(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128,                                      -cos(theta1), (81129638414606686663546605165575*self.realsense_l4*sin(theta1 + theta2 + theta3 + theta4))/162259276829213363391578010288128 - (81129638414606676728031405122553*self.l3*sin(theta2 - theta1 + theta3))/162259276829213363391578010288128 + (81129638414606686663546605165575*self.l2*sin(theta1 + theta2))/162259276829213363391578010288128 - (81129638414606676728031405122553*self.realsense_l4*sin(theta2 - theta1 + theta3 + theta4))/162259276829213363391578010288128 + (81129638414606676728031405122553*self.l2*sin(theta1 - theta2))/162259276829213363391578010288128 + (81129638414606686663546605165575*self.l3*sin(theta1 + theta2 + theta3))/162259276829213363391578010288128],
        [                                                                                                                                                                                             sin(theta2 + theta3 + theta4),                                                                                                                                                                                               cos(theta2 + theta3 + theta4), 4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              self.l1 + self.l3*sin(theta2 + theta3) + self.l2*sin(theta2) + self.realsense_l4*sin(theta2 + theta3 + theta4)],                                     
        [                                                                                                                                                                                 0,                                                                                                                                                                                                                           0,                                                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     1]
                    ])

        return Trans

    def inverse_kinematics(self,X,Y,Z,theta):
        theta1=atan2(Y,X)
        A=X-self.l4*cos(theta1)*cos(theta)
        B=Y-self.l4*sin(theta1)*cos(theta)
        C=Z-self.l1-self.l4*sin(theta)
        K=A**2+B**2+C**2-self.l2**2-self.l3**2
       ##check domain
        if  abs(K/2/self.l2/self.l3) < 1:

            theta3 = acos(K/2/self.l2/self.l3)
            a=self.l3*sin(theta3)
            b=self.l2+self.l3*cos(theta3)
            c=Z-self.l1-self.l4*sin(theta)
            r=sqrt(a**2+b**2)
            theta2 = atan2(c,sqrt(r**2-c**2))-atan2(a,b)
            theta4 = theta-theta2-theta3
            #theta2 = pi/2-(atan2(c,sqrt(r**2-c**2))-atan2(a,b))
            #theta4 = pi/2-(theta-theta2-theta3)

            theta1 = int(theta1 * 180/pi)
            theta2 = int(theta2 * 180/pi)
            theta3 = int(theta3 * 180/pi)
            theta4 = int(theta4 * 180/pi)
            
            theta4 = 3*pi/4 - theta4
            #if (theta4+135) >180 :     #check
            #    theta4 = theta4-270

            theta_all=[theta1,theta2,theta3,theta4]
            for i in theta_all: 
                if i > 180 or i < 0:
                    the = 0
                    break
                if i == theta4:
                    the = str(theta1) + "," + str(theta2) + "," + str(theta3) + "," + str(theta4) + "," + str(555) +  "\n"
            #self.forward_kinematics(theta1, theta2, theta3, theta4, self.realsensense_l4)
            return the
        else:
            return 0

    def rotation_matrix(self,xyz,rad):
        if xyz == "x":
            trans = np.array([[1,0,0,0],[0,cos(rad),-sin(rad),0],[0,sin(rad),cos(rad),0],[0,0,0,1]])
        if xyz == "y":
            trans = np.array([[cos(rad),0,sin(rad),0],[0,1,0,0],[-sin(rad),0,cos(rad),0],[0,0,0,1]])
        if xyz == "z":
            trans = np.array([[cos(rad),-sin(rad),0,0],[sin(rad),cos(rad),0,0],[0,0,1,0],[0,0,0,1]])

        return trans

if __name__=="__main__":
    a = Robot_control()
    TRANS = a.forward_kinematics(pi/2 , pi/2 , pi/2 , 0)
    print(TRANS)
