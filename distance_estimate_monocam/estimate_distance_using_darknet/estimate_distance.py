####################################################################################
#
#	모노카메라를 통한 거리검출(예측)
#
#	작성자 : 백장현 
#	수정일 : 2020/09/22
#
#	Estimate distance using mono-camera
#	based on darknet (darknet_video.py)
#
#	yolov3 , yolov3-tiny ,yolov4 , yolov4-tiny 사용가능
#
#	gen_matrix_using_matrix를 통해 얻어온 matrix 의 abcdef값을 기반으로 object의 거리를 예측한다.
#	
#		
####################################################################################


from ctypes import *
import random
import os
import cv2
import time
import darknet
import argparse
from threading import Thread, enumerate
from queue import Queue

#	경로 설정
def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default=0,
                        help="video source. If empty, uses webcam 0 stream")
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")
    parser.add_argument("--weights", default="yolov3.weights",
                        help="yolo weights path")
    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")
    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--config_file", default="./cfg/yolov3.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./cfg/coco.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")
    return parser.parse_args()


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def set_saved_video(input_video, output_video, size):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    fps = int(input_video.get(cv2.CAP_PROP_FPS))
    video = cv2.VideoWriter(output_video, fourcc, fps, size)
    return video

#####################################################################
# modify here
# modify for test videos 
# 여기서부터 작성
#####################################################################

# video capture 함수 - usb cam으로 받아오는 영상 또는 저장되어있는 영상을 darknet yolo기반으로 confirm할수 있도록 변환해준
def video_capture(frame_queue, darknet_image_queue):

    ################using video file#############
    capture = cv2.VideoCapture('./data/dist_test_02.mp4')
    
    ################check usb cam -code#########
    ## v4l2-ctl -d /dev/video1 --list-formats
    ################using usbcam################
    # capture = cv2.VideoCapture(0)
    # capture = cv2.VideoCapture(1)
    ############################################
    while capture.isOpened():
        ret, frame = capture.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height),interpolation=cv2.INTER_LINEAR)
        frame_queue.put(frame_resized)
        darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())
        darknet_image_queue.put(darknet_image)
    capture.release()

    
# detection 함수  
def inference(darknet_image_queue, detections_queue, fps_queue):
    while cap.isOpened():
        darknet_image = darknet_image_queue.get()
        prev_time = time.time()
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
        detections_queue.put(detections)
        fps = int(1/(time.time() - prev_time))
        fps_queue.put(fps)
        print("FPS: {}".format(fps))
        darknet.print_detections(detections, args.ext_output)
    cap.release()

# drawing bounding box & infomation 
def drawing(frame_queue, detections_queue, fps_queue):
    random.seed(3)  # deterministic bbox colors
    # video = set_saved_video(cap, args.out_filename, (width, height))
#영상의 사이즈를 640*480으로 설정
    video = set_saved_video(cap, args.out_filename, (640, 480))
    while cap.isOpened():
        frame_resized = frame_queue.get()
        detections = detections_queue.get()
        fps = fps_queue.get()
        if frame_resized is not None:
            image = darknet.draw_boxes(detections, frame_resized, class_colors)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            if args.out_filename is not None:
                video.write(image)
            if not args.dont_show:
                cv2.imshow('Inference', image)
            if cv2.waitKey(fps) == 27:
                break
    cap.release()
    video.release()
    cv2.destroyAllWindows()

###############################################################################

if __name__ == '__main__':
    frame_queue = Queue()
    darknet_image_queue = Queue(maxsize=1)
    detections_queue = Queue(maxsize=1)
    fps_queue = Queue(maxsize=1)

    args = parser()
    check_arguments_errors(args)
    network, class_names, class_colors = darknet.load_network(
            args.config_file,
            args.data_file,
            args.weights,
            batch_size=1
        )
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    # width = darknet.network_width(network)
    # height = darknet.network_height(network)

    ##########modify here for default width /height
    width = 640
    height = 480
    darknet_image = darknet.make_image(width, height, 3)
    # input_path = str2int(args.input)
    # cap = cv2.VideoCapture(input_path)

    ###############using video file#################
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(1)
    ################using webcam####################
    # cap = cv2.VideoCapture('./data/dist_test_02.mp4')
    ################################################
    Thread(target=video_capture, args=(frame_queue, darknet_image_queue)).start()
    Thread(target=inference, args=(darknet_image_queue, detections_queue, fps_queue)).start()
    Thread(target=drawing, args=(frame_queue, detections_queue, fps_queue)).start()
