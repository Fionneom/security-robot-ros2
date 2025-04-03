"""\
ROS 2 node which uses the MobileNet computer vision model to detect humans and send the image via Discord
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

import requests

DISCORD_WEBHOOK_URL = "https://discord.com/api/webhooks/1352313250280378518/8ZPTXakmR7LysgmXsGtuxuy8z__8Tv3iyKaD5g18MSteUbupZakYDsItmlORTpDe24H5"

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.bridge = CvBridge()

        # Uses compressed image to improve speed
        self.subscription = self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.image_callback,10)

        # 
        package_dir = get_package_share_directory('sec_bot_detection_ros2')
        self.prototxt_path = os.path.join(package_dir, 'models', 'deploy.prototxt')
        self.model_path = os.path.join(package_dir, 'models', 'mobilenet_iter_73000.caffemodel')

        self.net = cv2.dnn.readNetFromCaffe(self.prototxt_path, self.model_path)
        self.get_logger().info('Model loaded successfully')

        # Model can be used to differentiate many objects, only humans being detected currently
        self.classes = {15: 'person'}

        self.person_count = 0

        self.confidence_threshold = 0.75

    def image_callback(self, msg):
        try:
            # Decompress image to cv
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error("Failed to decode image.")
                return

            (h, w) = cv_image.shape[:2]

            blob = cv2.dnn.blobFromImage(cv_image, 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()

            human_count = 0

            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence > self.confidence_threshold:  # Confidence threshold
                    class_id = int(detections[0, 0, i, 1])

                    if class_id in self.classes and self.classes[class_id] == 'person':
                        human_count += 1

                        # Get bounding box
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")

                        # Draw bounding box
                        cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)

            # If the number of humans detected has changed, send a notification
            if human_count != self.person_count and human_count > 0:
                cv2.imwrite('./image.jpg', cv_image)
                self.send_notification()

            self.person_count = human_count

            # Show the video feed
            cv2.imshow('Human Detection', cv_image)
            cv2.waitKey(1)

            self.get_logger().info('Detected ' + str(human_count) +' human(s).')

        except Exception as e:
            self.get_logger().error('Error processing image: ' + str(e))


    def send_notification(self):
        # Sends image to Discord via Webhook
        data = {"content": "Person Detected"}
        files = {"file": open("./image.jpg", "rb")}
        requests.post(DISCORD_WEBHOOK_URL, data = data, files = files)


def main(args=None):
    rclpy.init(args=args)

    human_detector = HumanDetector()

    try:
        rclpy.spin(human_detector)
    except KeyboardInterrupt:
        pass
    finally:
        human_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
