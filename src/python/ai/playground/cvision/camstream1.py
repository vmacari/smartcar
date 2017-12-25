import cv2


#import cv2
import cv2 as cv
import numpy as np



class Lines():

    minEdge = 200
    maxEdge = 300
    minLineLength = 1000
    maxLineGap = 100
    p = 1
    theta = np.pi/180
    threshhold = 200
    probability = False

    def __init__(self):
        pass

    def getLines(self, grayImage):
        edges = cv2.Canny(grayImage, self.minEdge, self.maxEdge, apertureSize=3, L2gradient=True)

        if self.probability:
            lines = cv2.HoughLinesP(edges, self.p, self.theta, self.threshhold, self.minLineLength, self.maxLineGap)
        else:
            lines = cv2.HoughLines(edges, self.p, self.theta, self.threshhold)

        return lines


    def drawLines(self, targetFrame, grayImage):

        lines = self.getLines(grayImage)

        if self.probability:
            if lines is None:
                pass
                #maxLineGap *= .99
                #minLineLength *= 1.01
            else:
                for x1, y1, x2, y2 in lines[0]:
                    cv2.line(targetFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            if lines is None:
                pass
                #maxLineGap *= .99
                #minLineLength *= 1.01
            else:
                for rho, theta in lines[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))

                    cv2.line(targetFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)



class Circles():

    accumulator_threshold = 75
    minCircleDist = 100
    maxCannyThresh = 175
    minRadius = 10
    maxRadius = 0

    def __init__(self):
        pass

    def getCircles(self, img):

        smoothed = cv2.medianBlur(img, 5)
        circles = cv2.HoughCircles(smoothed, cv.HOUGH_GRADIENT, 1
                                   , self.minCircleDist
                                   , param1=self.maxCannyThresh
                                   , param2=self.accumulator_threshold
                                   , minRadius=self.minRadius
                                   , maxRadius=self.maxRadius)
        if circles is None:
            return None

        circles = np.uint16(np.around(circles))
        return circles

    def drawCircles(self, targetFrame, grayFrame):

        circles = self.getCircles(grayFrame)

        if circles is None:
            return

        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(targetFrame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(targetFrame, (i[0], i[1]), 2, (0, 0, 255), 3)



if __name__ == '__main__':


    cap = cv2.VideoCapture(0)
    #cap.open('rtmp://flash.oit.duke.edu/vod/_definst_/test/Wildlife2.flv')
    cap.open('rtsp://admin:@192.168.0.10:554/user=admin_password=_channel=1_stream=0.sdp?real_stream')

    circlesObj = Circles()
    linesObj = Lines()

    '''Main loop to read from camera'''
    while(True):

        # Capture frame-by-frame
        try:
            ret, frame = cap.read()

            # Gray scale the frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # #Detect and draw circles
            # circlesObj.drawCircles(frame, gray)
            #
            # #detect and draw lines
            # linesObj.drawLines(frame, gray)

            # Display the resulting frame

            height, width, layers = frame.shape
            new_h = height / 2
            new_w = width / 2
            frame = cv2.resize(frame, (new_w, new_h))

            cv2.imshow('Video Feed processed', frame)
        except:
            pass


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
