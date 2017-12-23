import numpy as np
import cv2


if __name__ == '__main__':

    cap = cv2.VideoCapture(0)
    cap.open(
        'rtsp://admin:@192.168.0.10:554/user=admin_password=_channel=1_stream=0.sdp?real_stream')

    fgbg = cv2.createBackgroundSubtractorMOG2(
        detectShadows=True)  # Create the background substractor

    kernelOp = np.ones((3, 3), np.uint8)
    kernelCl = np.ones((11, 11), np.uint8)
    areaTH_l = 500


    while (cap.isOpened()):
        ret, frame = cap.read()  # read a frame

        fgmask = fgbg.apply(frame)  # Use the substractor
        try:
            ret, imBin = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)
            # Opening (erode->dilate) para quitar ruido.
            mask = cv2.morphologyEx(imBin, cv2.MORPH_OPEN, kernelOp)
            # Closing (dilate -> erode) para juntar regiones blancas.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelCl)
        except:
            # if there are no more frames to show...
            print('EOF')
            break

        _, contours0, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_NONE)
        for cnt in contours0:
            #cv2.drawContours(frame, cnt, -1, (0, 255, 0), 3, 8)
            area = cv2.contourArea(cnt)
            print ('Area : {}'.format(area))
            if area > areaTH_l:
                #################
                #   TRACKING    #
                #################
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                x, y, w, h = cv2.boundingRect(cnt)
                #cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                img = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


        height, width, layers = frame.shape
        new_h = height / 2
        new_w = width / 2
        frame = cv2.resize(frame, (new_w, new_h))

        cv2.imshow('Frame', frame)

        # Abort and exit with 'Q' or ESC
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()  # release video file
    cv2.destroyAllWindows()  # close all openCV windows
