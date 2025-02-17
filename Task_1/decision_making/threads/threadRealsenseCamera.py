# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import cv2
import numpy as np
import threading
import base64
import time
import line_profiler
import sys

import pyrealsense2 as rs
from .hailoRunner import HailoInference as Infer
from src.utils.detector import Detector

from src.bridge.connection import Sender
from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Contrast,
    Brightness,
    ParkingMarker,
    CurrentSpeed,
    CurrentSteer
)

from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop


class threadRealsenseCamera(ThreadWithStop):
    """Thread which will handle camera functionalities.\n
    Args:
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadRealsenseCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.record = False
        self.debugInference = True

        self.Subscriber()
        self.Sender()
        self._init_camera()


        mainRequest, _ = self.readFrames
        self.ratio = 1
        self.height, self.width, _ = mainRequest.shape

        if self.record:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter('output.mp4', fourcc, 30.0, (self.width, self.height))

        # self.cropMaskSize = 0
        # halfLine = self.width // 2 * self.ratio
        # upperPoint = np.array([40, self.height * self.ratio - self.cropMaskSize - 180])
        # lowerPoint = np.array([0, self.height * self.ratio - self.cropMaskSize])
        # symmetricUpper = np.array([halfLine - upperPoint[0] + halfLine, upperPoint[1]])
        # symmetricLower = np.array([halfLine - lowerPoint[0] + halfLine, lowerPoint[1]])

        # self.srcPoints = np.array([upperPoint, 
        #                            symmetricUpper,
        #                            symmetricLower, 
        #                            lowerPoint], dtype = np.float32)
        self.createWarpPoint()    
        
        
        self.stopEvent = threading.Event() 
        if self.debugInference:
            self.AIInference = Infer("./src/models/yolov8n_seg.hef")
            
        self.laneGuidance = Detector(self.srcPoints)
        self.feedSender = Sender("0.0.0.0", 8000)
        self.debugFeedSender = Sender("0.0.0.0", 7999)
        self.laneGuidanceSender = Sender("0.0.0.0", 8001)
        
    def createWarpPoint(self):
        self.cropMaskSizeUpper = 40
        self.cropMaskSizeLower = 40
        self.pad = 0
        
        halfLine = 160 // 2 * self.ratio
        upperPoint = np.array([0, 160 - self.cropMaskSizeUpper - self.cropMaskSizeLower - 40])
        lowerPoint = np.array([0, 160 - self.cropMaskSizeUpper - self.cropMaskSizeLower + self.pad])
        symmetricUpper = np.array([halfLine - upperPoint[0] + halfLine, upperPoint[1]])
        symmetricLower = np.array([halfLine - lowerPoint[0] + halfLine, lowerPoint[1]])

        self.srcPoints = np.array([upperPoint, 
                                   symmetricUpper,
                                   symmetricLower, 
                                   lowerPoint], dtype = np.float32)
    
    def Sender(self):
        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.parkingMarkerSender = messageHandlerSender(self.queuesList, ParkingMarker)
        self.steerSender = messageHandlerSender(self.queuesList, CurrentSteer)
        self.speedSender = messageHandlerSender(self.queuesList, CurrentSpeed)

    def Subscriber(self):
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)

    # =============================== STOP ================================================
    def stop(self):
        """Stop the thread and clean up resources."""
        self.stopEvent.set()  # Signal the thread to stop
        if self.record:
            self.writer.release()

        self.realsensePipeline.stop()
        if self.debugInference:
            self.AIInference.stop
        print("\033[92m[INFO]\033[0m Realsense Pipeline has been closed")
        self.feedSender.stop()
        super(threadRealsenseCamera, self).stop()


    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True. It captures the image from camera and make the required modifies and then it send the data to process gateway."""

        send = True
        brightnessVal = 1

        prepareDocking = False
        parkingMarkerDetMode = False; prepareMarkerDet = False; timerStarted = False; avoidMode = False
        speed = 160
        possibleParkingSpot = 2; parkingIdx = -1
        
        while self._running:
            if send:
                brightnessValPlaceholder = self.brightnessSubscriber.receive()
                if brightnessValPlaceholder is not None:
                    brightnessVal = brightnessValPlaceholder
                
                start = time.time()
                mainRequest, (depthImage, depthColormap) = self.readFrames
                mainRequest = self.adjust_contrast(mainRequest, alpha = brightnessVal)

                if self.debugInference:

                    mainRequest = cv2.resize(mainRequest, (int(self.width * self.ratio), int(self.height * self.ratio)))
                    padSz = abs(mainRequest.shape[1] - mainRequest.shape[0])
                    mainRequest = np.pad(mainRequest, ((0, padSz), (0, 0), (0, 0)), mode = "constant", constant_values = 0)
                    mainRequest = cv2.resize(mainRequest, (640, 640))

                    outputDetection = self.AIInference.inference(mainRequest[:, :, ::-1])
                    mainRequest = mainRequest[:self.height, :self.width]
                    
                    
                    angle = 90
                    if outputDetection != None:

                        boxes = (outputDetection["detection_boxes"] * 640).astype(int)
                        masks = outputDetection['mask']
                        classIDs = outputDetection['detection_classes']
                        scores = outputDetection["detection_scores"]

                        laneMasksPlane = np.zeros((np.sum(classIDs == 4), 160 - self.cropMaskSizeUpper - self.cropMaskSizeLower + self.pad, 160), dtype = np.float64)
                        parkingSpotBoxes, carsCenter = [], []
                        maskIndex = 0
                        for boxCoor, maskPlane, classID, score in zip(boxes, masks, classIDs, scores):

                            if classID != 4:
                                className = list(self.AIInference.modelArgs["labels"][classID].keys())[0]

                                # Approximated alignment
                                shiftedBox = self.boxSift(boxCoor, [0.62, 0.62], [-10, 5], [self.width // 2, self.height // 2])
                                shiftedCenter = ((shiftedBox[2] + shiftedBox[0]) // 2, (shiftedBox[3] + shiftedBox[1]) // 2)
                                center = ((boxCoor[2] + boxCoor[0]) // 2, (boxCoor[3] + boxCoor[1]) // 2)

                                cv2.circle(mainRequest, (center[0], center[1]), 3, (0, 255, 0), cv2.FILLED)
                                cv2.circle(depthColormap, (shiftedCenter[0], shiftedCenter[1]), 3, (0, 255, 0), cv2.FILLED)

                                depthBox = depthImage[shiftedBox[1]: shiftedBox[3], shiftedBox[0]: shiftedBox[2]]
                                distance = np.unique(depthBox)[1] if np.all(depthBox == 0) == False else 0                               

                                # distance = depthImage[shiftedCenter[1], shiftedCenter[0]]
                                
                                text = f"{className}: {score:.2f}, {distance} (mm)"
                                self.AIInference.drawBox(mainRequest, boxCoor, text)
                                self.AIInference.drawBox(depthColormap, shiftedBox, "")

                                # Parking
                                if prepareDocking:
                                    if classID == 0: carsCenter += [np.concatenate((center, boxCoor))]
                                    if classID == 9: parkingSpotBoxes += [np.concatenate((center, boxCoor))]
                                    
                                if classID == 8 and 10 <= distance <= 520:
                                    print(distance)
                                    speed = 150
                                    prepareDocking = True
                                
                                if classID == 0 and 10 <= distance <= 600 and (250 <= center[0] <= 390 and 190 <= center[1] <= 290):
                                    avoidMode = True

                            maskThresh = 0.9
                            if classID == 4:
                                # maskPlane = maskPlane[self.cropMaskSize: self.height, :self.width]
                                # maskPlane = self.AIInference.fasterMaskFilter(boxCoor, maskPlane, maskThresh)
                                maskPlane[maskPlane < maskThresh] = 0
                                laneMasksPlane[maskIndex] = np.pad(maskPlane[self.cropMaskSizeUpper: 160 - self.cropMaskSizeLower], ((0, self.pad), (0, 0)))
                                maskIndex += 1

                                self.AIInference.drawMask(mainRequest, maskPlane = cv2.resize(maskPlane, (640, 640))[: 480])


                        if prepareDocking and prepareMarkerDet == False:
                            
                            if len(parkingSpotBoxes) < possibleParkingSpot and carsCenter:
                                parkingSpotBoxes += [car for car in carsCenter] # assume car is on parking lot
                            
                            parkingSpotBoxes = sorted(parkingSpotBoxes, key = lambda x: x[1], reverse = True)
                            
                            if len(parkingSpotBoxes) == possibleParkingSpot:
                                
                                for parkingIdx, parkingSpot in enumerate(parkingSpotBoxes):
                                    centerX, centerY, x1, y1, x2, y2 = parkingSpot
                                    found = False
                                    if len(carsCenter) == 0:
                                        prepareMarkerDet = True
                                        break
                                    
                                    for carCenter in carsCenter:
                                        if not (x1 <= carCenter[0] <= x2 and y1 <= carCenter[1] <= y2):    
                                            prepareMarkerDet = True
                                            found = True
                                            break
                                    if found:
                                        break
                            
                        # print(f"Befor lane guide: {1 / (time.time() - start):.2f}", end = " ")
                        allLanes = np.any(laneMasksPlane, axis=0).astype(np.uint8) * 255
                        if maskIndex != 0:
                            angle, debugImage = self.laneGuidance.guide(laneMasksPlane)
                        self.laneGuidance.directionDraw(mainRequest, angle * np.pi / 180, 50, (self.width // 2, self.height))
                        self.laneGuidance.pointDrawer(self.srcPoints, allLanes)
                        

                if self.record:
                    self.writer.write(mainRequest)

                if prepareMarkerDet == True:
                    if timerStarted == False:
                        startTimer = time.time()
                        timerStarted = True
                    
                    if time.time() - startTimer >= 2:
                        parkingMarkerDetMode = True
                

                self.feedSender.sendMessage(allLanes)
                self.debugFeedSender.sendMessage(mainRequest)
                self.laneGuidanceSender.sendMessage(f"speed: {speed}|steer: {angle}|avoidMode: {avoidMode}")
                self.parkingMarkerSender.send(parkingIdx if parkingMarkerDetMode == True else -1)
                avoidMode = False



                # _, mainEncodedImg = cv2.imencode(".png", np.dstack((mainRequest.astype(depthImage.dtype), depthImage)))
                # _, serialEncodedImg = cv2.imencode(".jpg", serialRequest)

                # mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
                # serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

                # self.mainCameraSender.send(mainEncodedImageData)
                # self.serialCameraSender.send(serialEncodedImageData)
            
                print(parkingIdx)
                print(f"Entire proc: {1 / (time.time() - start):.2f}", end = "\n")
                    

            send = not send

    # =============================== START ===============================================
    def start(self):
        super(threadRealsenseCamera, self).start()

    # ================================ INIT CAMERA ========================================
    def _init_camera(self):
        """This function will initialize the camera object. It will make this camera object have two chanels "lore" and "main"."""

        self.realsensePipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.realsensePipeline.start(config)

        # self.align = rs.align(rs.stream.color)
    
    # =============================== READ FRAMES =========================================
    @property
    def readFrames(self):
        frames = self.realsensePipeline.wait_for_frames()

        # alignFrames = self.align.process(frames)

        # depth_frame = alignFrames.get_depth_frame()
        # color_frame = alignFrames.get_color_frame()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame: return None, (None, None)

        depthImage = np.asanyarray(depth_frame.get_data())
        colorImage = np.asanyarray(color_frame.get_data())

        depthColormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depthImage, alpha = 0.1),
            cv2.COLORMAP_JET
        )

        return colorImage, (depthImage, depthColormap)
    
    @staticmethod    
    def adjust_contrast(image, alpha=1.5, beta=0):
        """
        Adjusts contrast and brightness of an image.
        alpha: Contrast control (1.0 - 3.0)
        beta: Brightness control (0 - 100)
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    
    @staticmethod
    def boxSift(boxCoor: np.ndarray, velocity: np.ndarray, shiftVal: np.ndarray, halfLine: np.ndarray):
        """boxCoor format: xyxy | velocity format: xy | shiftVal format: xy | haflLine format: xy"""
        shiftedBox = boxCoor.copy()
        shiftedBox[0] = (shiftedBox[0] - halfLine[0]) * velocity[0] + halfLine[0] + shiftVal[0]
        shiftedBox[2] = (shiftedBox[2] - halfLine[0]) * velocity[0] + halfLine[0] + shiftVal[0]
        shiftedBox[1] = (shiftedBox[1] - halfLine[1]) * velocity[1] + halfLine[1] + shiftVal[1]
        shiftedBox[3] = (shiftedBox[3] - halfLine[1]) * velocity[1] + halfLine[1] + shiftVal[1]
        return shiftedBox