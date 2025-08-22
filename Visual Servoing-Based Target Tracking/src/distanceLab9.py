#!/usr/bin/python3

import rospy
import time
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class TakePhoto:
    def __init__(self):
        self.image_received = False
        img_topic = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size=1)

    def callback(self, data):
        self.image_received = True
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def get_img(self):
        if not self.image_received:
            print("None")
            return
        return self.img

    def save_img(self, img_title):
        if not self.image_received:
            print("None")
            return
        cv2.imwrite(img_title, self.img)

    def disp_img(self, img_title):
        if not self.image_received:
            print("None")
            return
        cv2.imshow(img_title, self.img)


class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)

    def callback(self, cmd):
        self.cmd = cmd.data

    def get_msg(self):
        return self.cmd


DIMENSION = (480 , 640)

if __name__ == "__main__":
    i = 0
    rospy.init_node("CameraProcessing")
    camera = TakePhoto()
    cmd_listener = CmdListener()
    rate = rospy.Rate(10)
    led_on = None
    led_off = None
    time.sleep(3)

    while not rospy.is_shutdown():
        while cmd_listener.get_msg() == "HIGH":
            pass
        while cmd_listener.get_msg() == "LOW":
            pass

        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.45)
            led_on = camera.get_img()
            while cmd_listener.get_msg() == "HIGH":
                pass

        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.45)
            led_off = camera.get_img()
            while cmd_listener.get_msg() == "LOW":
                pass

        if led_on is not None and led_off is not None:
            diff = cv2.absdiff(led_on, led_off)
            diffB = 2 * cv2.blur(diff, (3, 3))
            mx_diffB = np.amax(diffB)
            print("max of Blured ", mx_diffB)
            print("Blured diffB shape")
            print(diffB.shape)
            cv2.imshow("Diff", diffB)

            NX = 30
            idx = np.argmax(diffB, axis=0)
            val = np.amax(diffB, axis=0)
            ival = np.argsort(val)[::-1]
            diffV = np.reshape(diffB, [640 * 480])
            AI = np.flip(np.argsort(diffB, axis=None))[0:NX]
            AZ = np.zeros([NX, 5], dtype=np.int32)
            AZ[:, 0] = diffV[AI]
            AZ[:, 1] = AI
            AZ[:, 2] = AI / 640
            AZ[:, 3] = AI % 640

            AZ2i = np.argsort(AZ[:, 3])
            AZZ = AZ[AZ2i, :]

            clus = 1
            pt0 = 0
            for i in range(NX):
                if (AZZ[i, 3] - AZZ[pt0, 3]) < 2:
                    AZZ[i, 4] = clus
                else:
                    clus += 1
                    AZZ[i, 4] = clus
                pt0 = i

            print(AZZ)
            led_on = 0
            led_off = 0
            i += 1

            Clust = np.zeros((clus, 7), dtype=np.double)
            for i in range(clus):
                Clust[i, 0] = i + 1
                clus_ind = np.where(AZZ[:, 4] == [i + 1])[0]
                print(i + 1)
                print(clus_ind)

                Clust[i, 1] = np.dot(AZZ[clus_ind, 3], AZZ[clus_ind, 0]) / np.sum(AZZ[clus_ind, 0])
                Clust[i, 2] = np.dot(AZZ[clus_ind, 2], AZZ[clus_ind, 0]) / np.sum(AZZ[clus_ind, 0])
                Clust[i, 3] = np.amin(AZZ[clus_ind, 3])
                Clust[i, 4] = np.amax(AZZ[clus_ind, 3])
                Clust[i, 5] = np.amin(AZZ[clus_ind, 2])
                Clust[i, 6] = np.amax(AZZ[clus_ind, 2])

                print("X", Clust[i, 1], " Xmin", Clust[i, 3], "Xmax", Clust[i, 4])
                print("Y", Clust[i, 2], " Ymin", Clust[i, 5], "Ymax", Clust[i, 6])
                print()

            print("---------------------------------------------------------")
            print("Left Front LED Coordinate  ", Clust[0, 1], ", ", Clust[0, 2])
            print("Right Front LED Coordinate ", Clust[2, 1], ", ", Clust[2, 2])
            print("Rear LED Coordinate        ", Clust[1, 1], ", ", Clust[1, 2])
            print("---------------------------------------------------------")

            cv2.waitKey(0)

        rate.sleep()
