import cv2

if __name__ == '__main__':
    cv2.namedWindow("camCapture", cv2.WINDOW_AUTOSIZE)
    cap = cv2.VideoCapture()


    cap.open('rtsp://admin:@192.168.0.10:554/user=admin_password=_channel=1_stream=0.sdp?real_stream')
    if not cap.open:
        print("Not open")
        exit(-1)

    while (True):
        err,img = cap.read()
        if img and img.shape != (0,0):
            cv2.imwrite("img1", img)
            cv2.imshow("camCapture", img)
        if err:
            print(err)
            break
        cv2.waitKey(30)
