#!/usr/bin/env python3

import sys
import numpy as np

import argparse
import torch
import cv2
import pyzed.sl as sl
from ultralytics import YOLO

from threading import Lock, Thread
from time import sleep

import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer

# mrr
import rospy
from std_msgs.msg import String
import math


lock = Lock()
run_signal = False
exit_signal = False


def calculate_rectangle_lines(top_left, bottom_right):
    x1, y1 = top_left
    x2, y2 = bottom_right

    top_line = [(x1, y1), (x2, y1)]
    bottom_line = [(x1, y2), (x2, y2)]
    left_line = [(x1, y1), (x1, y2)]
    right_line = [(x2, y1), (x2, y2)]

    return top_line, bottom_line, left_line, right_line

def get_obstacle_str(line):
    line_x1, line_y1 = line[0]
    line_x2, line_y2 = line[1]
    return str(int(line_x1)) + "\n" + str(int(line_y1)) + "\n" + str(int(line_x2)) + "\n" + str(int(line_y2)) + "\nring\n"


def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = (xywh[0] - 0.5*xywh[2]) #* im_shape[1]
    x_max = (xywh[0] + 0.5*xywh[2]) #* im_shape[1]
    y_min = (xywh[1] - 0.5*xywh[3]) #* im_shape[0]
    y_max = (xywh[1] + 0.5*xywh[3]) #* im_shape[0]

    # A ------ B
    # | Object |
    # D ------ C

    output[0][0] = x_min
    output[0][1] = y_min

    output[1][0] = x_max
    output[1][1] = y_min

    output[2][0] = x_min
    output[2][1] = y_max

    output[3][0] = x_max
    output[3][1] = y_max
    return output

def detections_to_custom_box(detections, im0):
    output = []
    for i, det in enumerate(detections):
        xywh = det.xywh[0]

        # Creating ingestable objects for the ZED SDK
        obj = sl.CustomBoxObjectData()
        obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
        obj.label = det.cls
        obj.probability = det.conf
        obj.is_grounded = False
        output.append(obj)
    return output


def torch_thread(weights, img_size, conf_thres=0.2, iou_thres=0.45):
    global image_net, exit_signal, run_signal, detections, det, names

    print("Intializing Network...")

    model = YOLO(weights)

    while not exit_signal:
        if run_signal:
            lock.acquire()

            img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)
            # https://docs.ultralytics.com/modes/predict/#video-suffixes
            det = model.predict(img, save=False, imgsz=img_size, conf=conf_thres, iou=iou_thres)[0].cpu().numpy().boxes
            names = model.names
            # ZED CustomBox format (with inverse letterboxing tf applied)
            detections = detections_to_custom_box(det, image_net)
            lock.release()
            run_signal = False
        sleep(0.01)


def main():
    start_point_x = 29
    start_point_y = 90
    target_point_x = 120
    target_point_y = 204

    global image_net, exit_signal, run_signal, detections, det, names

    capture_thread = Thread(target=torch_thread, kwargs={'weights': opt.weights, 'img_size': opt.img_size, "conf_thres": opt.conf_thres})
    capture_thread.start()

    print("Initializing Camera...")

    zed = sl.Camera()

    input_type = sl.InputType()
    if opt.svo is not None:
        input_type.set_from_svo_file(opt.svo)

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 50

    runtime_params = sl.RuntimeParameters()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    image_left_tmp = sl.Mat()

    print("Initialized Camera")

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
    # positional_tracking_parameters.set_as_static = True
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    zed.enable_object_detection(obj_param)

    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    # Display
    camera_infos = zed.get_camera_information()
    camera_res = camera_infos.camera_configuration.resolution
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    point_cloud_res = sl.Resolution(min(camera_res.width, 720), min(camera_res.height, 404))
    point_cloud_render = sl.Mat()
    viewer.init(camera_infos.camera_model, point_cloud_res, obj_param.enable_tracking)
    point_cloud = sl.Mat(point_cloud_res.width, point_cloud_res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
    image_left = sl.Mat()
    # Utilities for 2D display
    display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
    image_scale = [display_resolution.width / camera_res.width, display_resolution.height / camera_res.height]
    image_left_ocv = np.full((display_resolution.height, display_resolution.width, 4), [245, 239, 239, 255], np.uint8)

    # Utilities for tracks view
    camera_config = camera_infos.camera_configuration
    tracks_resolution = sl.Resolution(400, display_resolution.height)
    track_view_generator = cv_viewer.TrackingViewer(tracks_resolution, camera_config.fps, init_params.depth_maximum_distance)
    track_view_generator.set_camera_calibration(camera_config.calibration_parameters)
    image_track_ocv = np.zeros((tracks_resolution.height, tracks_resolution.width, 4), np.uint8)
    # Camera pose
    cam_w_pose = sl.Pose()

    # mrr
    # Initialize the ROS node
    rospy.init_node('input_str_publisher', anonymous=True)

    # Create a publisher with topic name 'message_topic' and message type 'String'
    pub = rospy.Publisher('input_str', String, queue_size=10)
    # mrr

    while viewer.is_available() and not exit_signal:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # -- Get the image
            lock.acquire()
            zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
            image_net = image_left_tmp.get_data()
            lock.release()
            run_signal = True

            # -- Detection running on the other thread
            while run_signal:
                sleep(0.001)

            # Wait for detections
            lock.acquire()
            # -- Ingest detections
            zed.ingest_custom_box_objects(detections)
            lock.release()
            zed.retrieve_objects(objects, obj_runtime_param)

            # -- Display
            # Retrieve display data
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, point_cloud_res)
            point_cloud.copy_to(point_cloud_render)
            zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            zed.get_position(cam_w_pose, sl.REFERENCE_FRAME.WORLD)

            # 3D rendering
            viewer.updateData(point_cloud_render, objects)
            # 2D rendering
            np.copyto(image_left_ocv, image_left.get_data())
            cv_viewer.render_2D(image_left_ocv, image_scale, objects, obj_param.enable_tracking)
            global_image = cv2.hconcat([image_left_ocv, image_track_ocv])
            # Tracking view
            track_view_generator.generate_view(objects, cam_w_pose, image_track_ocv, objects.is_tracked)


            # mrr
            # is_ball_detected = False
            # for r in det:
            #     for c in r.cls:
            #         if int(c) == 32:
            #             is_ball_detected = True
            #             break
            #     if is_ball_detected:
            #         break
            #         # print(names[int(c)])
            # if is_ball_detected:
            obs_str = ""
            for square_object in objects.object_list:
                if square_object.raw_label == 32: # 32 is sports ball
                    # square_vertices = square_object.bounding_box_2d
                    square_center_vertices = square_object.position
                    print("Square Center Vertex Position (X, Y, Z) Meter:", square_center_vertices[0], square_center_vertices[1], square_center_vertices[2])

                    if not math.isnan(square_center_vertices[0]) and not math.isnan(square_center_vertices[1]):
                        # Center point coordinates (x, y)
                        center_x = int(square_center_vertices[0] * 100)
                        center_y = int (square_center_vertices[2] * -100)

                        # Side length of the square
                        side_length = 25

                        # Calculate the coordinates of the four vertices
                        top_left = (center_x - side_length/2, center_y - side_length/2)
                        bottom_right = (center_x + side_length/2, center_y + side_length/2)

                        top_line, bottom_line, left_line, right_line = calculate_rectangle_lines(top_left, bottom_right)
                        obs_str = obs_str + get_obstacle_str(top_line)
                        obs_str = obs_str + get_obstacle_str(bottom_line)
                        obs_str = obs_str + get_obstacle_str(left_line)
                        obs_str = obs_str + get_obstacle_str(right_line) 

                        # vertex_positions = []
                        # for vertex in square_vertices:
                        #     position = vertex
                        #     vertex_positions.append(position)
                        # print("Detected Square Vertex Positions:")
                        # for vertex_position in vertex_positions:
                        #     print("Vertex Position (X, Y): Pixel", vertex_position[0], vertex_position[1])
            if obs_str != "":
                start_point_str = str(start_point_x) + "\n" + str(start_point_y) + "\n"
                target_point_str = "target\n" + str(target_point_x) + "\n" + str(target_point_y) + "\nEND"
                input_str = start_point_str + obs_str + target_point_str
                # Publish the message
                pub.publish(input_str)
                # Print the published message
                rospy.loginfo('ROS Published message: %s', input_str)
            # mrr

            cv2.imshow("ZED | 2D View and Birds View", global_image)
            key = cv2.waitKey(10)
            if key == 27:
                exit_signal = True
        else:
            exit_signal = True

    viewer.exit()
    exit_signal = True
    zed.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='yolov8m.pt', help='model.pt path(s)')
    parser.add_argument('--svo', type=str, default=None, help='optional svo file')
    parser.add_argument('--img_size', type=int, default=416, help='inference size (pixels)')
    parser.add_argument('--conf_thres', type=float, default=0.65, help='object confidence threshold')
    opt = parser.parse_args()

    with torch.no_grad():
        main()
