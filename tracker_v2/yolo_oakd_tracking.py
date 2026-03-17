#!/usr/bin/env python3

import cv2
import depthai as dai
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import os

class YoloOakdTracking(Node):
    def __init__(self):
        super().__init__("yolo_oakd_tracking")
        self.fullFrameTracking = False
        self.show_window = False
        # self.model_path = "../models/model.rvc2.tar.xz"

        pkg_share = get_package_share_directory('tracker_v2')
        self.model_path = os.path.join(pkg_share, 'models', 'model.rvc2.tar.xz')

        self.pub = self.create_publisher(Float64MultiArray, "/spatial_pos", 10)
        
        with dai.Pipeline() as pipeline:
            self.start_pipeline(pipeline)
            while pipeline.isRunning():
                self.do_tracking()
    
    # Create pipeline
    def start_pipeline(self,pipeline):
        # Define sources and outputs
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        leftOutput = monoLeft.requestOutput((640, 400))
        rightOutput = monoRight.requestOutput((640, 400))
        leftOutput.link(stereo.left)
        rightOutput.link(stereo.right)

        nnArchive = dai.NNArchive(Path(self.model_path))
        spatialDetectionNetwork = pipeline.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, nnArchive)
        objectTracker = pipeline.create(dai.node.ObjectTracker)

        spatialDetectionNetwork.setConfidenceThreshold(0.6)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)
        labelMap = spatialDetectionNetwork.getClasses()

        objectTracker.setDetectionLabelsToTrack([0])  # track only person
        # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
        objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
        # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        self.preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
        self.tracklets = objectTracker.out.createOutputQueue()

        if self.fullFrameTracking:
            camRgb.requestFullResolutionOutput().link(objectTracker.inputTrackerFrame)
            # do not block the pipeline if it's too slow on full frame
            objectTracker.inputTrackerFrame.setBlocking(False)
            objectTracker.inputTrackerFrame.setMaxSize(1)
        else:
            spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)

        self.startTime = time.monotonic()
        self.counter = 0
        self.fps = 0
        self.color = (255, 255, 255)
        pipeline.start()
        self.get_logger().info("Tracking started")


            # while(pipeline.isRunning()):
            
    def do_tracking(self):
        imgFrame = self.preview.get()
        track = self.tracklets.get()
        assert isinstance(imgFrame, dai.ImgFrame), "Expected ImgFrame"
        assert isinstance(track, dai.Tracklets), "Expected Tracklets"

        self.counter+=1
        current_time = time.monotonic()
        if (current_time - self.startTime) > 1 :
            fps = self.counter / (current_time - self.startTime)
            self.counter = 0
            self.startTime = current_time

        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        for t in trackletsData:
            if self.show_window:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)

                try:
                    label = labelMap[t.label]
                except:
                    label = t.label
                cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                cv2.putText(frame, f"X: {(t.spatialCoordinates.x / 1000):.2f} m", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {(t.spatialCoordinates.y / 1000):.2f} m", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {(t.spatialCoordinates.z / 1000):.2f} m", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

        if trackletsData:
            sc = trackletsData[0].spatialCoordinates
            msg = Float64MultiArray()
            msg.data = [sc.x/1000.0, sc.y/1000.0, sc.z/1000.0]
            self.pub.publish(msg)
            self.get_logger().info(f"Detected object at: ({sc.x/1000:.2f}.{sc.y/1000:.2f},{sc.z/1000:.2f})")

        if self.show_window:
            cv2.putText(frame, "NN fps: {:.2f}".format(self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, self.color)
            cv2.imshow("tracker", frame)

def main(args=None):
    rclpy.init(args=args)
    node = YoloOakdTracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
