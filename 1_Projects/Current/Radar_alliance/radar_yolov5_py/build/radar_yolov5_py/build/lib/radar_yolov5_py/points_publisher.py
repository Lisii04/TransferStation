#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# USER_IMPORT
import array
import time
import cv2 as cv


# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
import argparse
import csv
import os
import platform
import sys
from pathlib import Path

import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode

from models.experimental import attempt_load
from utils.general import is_ascii, non_max_suppression
from utils.plots import Annotator, colors



class PointsPublisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("[start]%s" % name)
        self.command_publisher_ = self.create_publisher(Float32MultiArray,"points_data", 10) 
        
    def send_points(self,points):
        msg = Float32MultiArray()
        msg.data = points
        self.command_publisher_.publish(msg) 
        print("\033c>\033[33m[WORKING]\033[0m[正在发送点坐标]\033[?25l\n"+str(msg.data))


@smart_inference_mode()
def run(
        PointsPublisher_obj,
        car_weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        armor_weights = ROOT / 'yolov5s.pt',
        source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_csv=False,  # save results in CSV format
        save_conf=False,  # save confidences in --save-txt labels
        identfy_car=True,  # identfy which kind of robot 
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
):
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
    screenshot = source.lower().startswith('screen')
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(car_weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size


    # ============== armor_load ================
    armor_model = attempt_load(armor_weights,0)  # load FP32 armor_model
    # armor_names = armor_model.module.names if hasattr(armor_model, 'module') else armor_model.names  # get class names
    armor_names = ["armor_1_red","armor_2_red","armor_3_red","armor_4_red","armor_5_red","watcher_red","armor_1_blue","armor_2_blue","armor_3_blue","armor_4_blue","armor_5_blue","watcher_blue"]
    ascii = is_ascii(names)  # names are ascii (use PIL for UTF-8)
    # ============== armor_load ================


    # Dataloader
    bs = 1  # batch_size
    if webcam:
        view_img = check_imshow(warn=True)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        bs = len(dataset)
    elif screenshot:
        dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Define the path for the CSV file
        csv_path = save_dir / 'predictions.csv'

        # Create or append to the CSV file
        def write_to_csv(image_name, prediction, confidence):
            data = {'Image Name': image_name, 'Prediction': prediction, 'Confidence': confidence}
            with open(csv_path, mode='a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=data.keys())
                if not csv_path.is_file():
                    writer.writeheader()
                writer.writerow(data)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if identfy_car else im0  # for identfy_car
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):

                pointslist = []

                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = names[c] if hide_conf else f'{names[c]}'
                    confidence = float(conf)
                    confidence_str = f'{confidence:.2f}'

                    if save_csv:
                        write_to_csv(p.name, label, confidence_str)

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(f'{txt_path}.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or identfy_car or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        # label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        # annotator.box_label(xyxy, label, color=colors(c, True))
                    if identfy_car and label == "car":
                        # ===========================================
                        croped_image = imc[int(xyxy[1]):int(xyxy[3]),int(xyxy[0]):int(xyxy[2])]

                        if croped_image.shape[0] > 0 and croped_image.shape[1] > 0:
                            croped_image = cv.resize(croped_image,( (round(croped_image.shape[0]/32) )* 32 , ( round(croped_image.shape[1]/32) )* 32 ))
                            car_image = torch.from_numpy(croped_image).to(device)
                            car_image = car_image.to(torch.float32) # uint8 to fp32
                            car_image = car_image / 255.0  # 0 - 255 to 0.0 - 1.0
                            if len(car_image.shape) == 3:
                                car_image = car_image[None]  # expand for batch dim
                            car_image = car_image.transpose(2, 3)
                            car_image = car_image.transpose(1, 2)

                            # Inference
                            armor_pred = armor_model(car_image)[0]

                            # NMS
                            armor_pred = non_max_suppression(armor_pred, conf_thres = 0.25, iou_thres = 0.45, classes = None, max_det = 1000)

                            # Process predictions
                            for i, det in enumerate(armor_pred):  # detections per image
                                armor_s = ''
                                armor_annotator = Annotator(croped_image, line_width=line_thickness, example=str(names))
                                if len(det):
                                    # Rescale boxes from img_size to im0 size
                                    det[:, :4] = scale_boxes(car_image.shape[2:], det[:, :4], croped_image.shape).round()

                                    # Print results
                                    for c in det[:, -1].unique():
                                        n = (det[:, -1] == c).sum()  # detections per class
                                        armor_s += str(n.item()) + ' ' + str(armor_names[int(c)]) + ' '  # add to string

                                    score = {}
                                    # Write results
                                    for *armor_xyxy, conf, cls in reversed(det):
                                        c = int(cls)  # integer class

                                        if armor_names[c] in score:
                                            score[armor_names[c]] += conf
                                        else:
                                            score[armor_names[c]] = conf
                                    
                                    conf_max = 0
                                    armor_max = ''
                                    for armor_name in score:
                                        if score[armor_name] > conf_max:
                                            armor_max = armor_name
                                            conf_max = score.get(armor_name)


                                    center_x = (xyxy[0] + xyxy[2]) / 2

                                    armor_label = armor_max
                                    annotator.box_label(xyxy, armor_label, color=colors(c, True))
                                    # armor_annotator.box_label(armor_xyxy, armor_label, color=colors(c, True))
                                    cv.putText(annotator.result(),armor_label,(int(center_x),(int(xyxy[3])+10)),1,3,(0,255,0),5)
                                    pointslist.append(armor_names.index(armor_label))
                                    # ===========================================                       
                                   
                                    # center_y = (xyxy[1] + xyxy[3]) / 2
                                    # if xyxy[1] > xyxy[3]:
                                    #     bottom_y = xyxy[1]
                                    # else:
                                    #     bottom_y = xyxy[3]
                                    pointslist.append(center_x)
                                    pointslist.append(xyxy[3])

                # points--------------------------------
                points = array.array('f',pointslist)
                PointsPublisher_obj.send_points(points)
                # points--------------------------------

            
            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

        # Print time (inference-only)
        # LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")
        print("\033[32m>[识别结果数量]\033[0m" + str(len(det)) + "\n\033[32m>[每一帧用时]\033[0m" + f"{dt[1].dt * 1E3:.1f}ms")

    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(car_weights[0])  # update model (to fix SourceChangeWarning)



def main(args=None):
    print("\033[1m[正在启动神经网络识别节点]\033[0m")
    print(">\033[33m[WORKING]\033[0m[初始化rclpy]")
    rclpy.init(args=args) # 初始化rclpy
    node = PointsPublisher("points_publisher")  # 新建一个节点
    print(">\033[32m[DONE]\033[0m[初始化完成]")
    print(">\033[32m[DONE]\033[0m[节点启动]")
    print(">\033[33m[WORKING]\033[0m[正在启动YOLOv5]")
    time.sleep(1)
    run(node,car_weights=os.getcwd()+'/user_models/car_identfy.pt',armor_weights=os.getcwd()+'/user_models/armor_identfy.pt',source=os.getcwd()+'/videos/1.mp4',nosave=True,view_img=True,identfy_car=True)

    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
   
# if __name__ == "__main__":
#     main()
