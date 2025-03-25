#!/usr/bin/env python3
# filepath: /home/sstevenson/ORS-24-25/robot_simulation/src/ors_robot/scripts/openai_scene_description.py

""" Running
export OPENAI_API_KEY="sk-..."
cd ~/ORS-24-25/robot_simulation/src/ors_robot/launch
python3 openai_scene_description.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import requests
from openai import OpenAI
import os
from threading import Event

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        
        # OpenAI API settings
        self.api_key = os.environ.get('OPENAI_API_KEY')
        if not self.api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError("OPENAI_API_KEY environment variable must be set")
        self.client = OpenAI()
        
        # Set up the OpenAI API endpoint
        self.api_url = "https://api.openai.com/v1/chat/completions"
        
        # Set default prompt
        self.prompt = "Describe what you see in this image."
        
        # Topics for different camera streams
        topic = '/depth_cam/image_raw'
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.camera_callback,
            10
        )
        
        # OpenCV bridge for converting ROS images to OpenCV format
        self.cv_bridge = CvBridge()
        
        # Event to signal when an image has been processed
        self.done_event = Event()
        
        # Flag to process the next frame
        self.process_next_frame = True
        
        # Debug mode - just save image without sending to API
        self.debug_mode = False  # Set to False when ready to use API
        
        self.get_logger().info(f"Subscribed to {topic}. Waiting for image...")

    def camera_callback(self, msg):
        if not self.process_next_frame:
            return
        
        self.get_logger().info("Received image frame")
        
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Save image temporarily
            temp_filename = "/tmp/camera_frame.jpg"
            cv2.imwrite(temp_filename, cv_image)
            
            self.get_logger().info(f"Image saved to {temp_filename}")
            
            # Debug information about the saved image
            img_info = cv2.imread(temp_filename)
            if img_info is not None:
                self.get_logger().info(f"Image dimensions: {img_info.shape}")
                self.get_logger().info(f"Image successfully saved and verified")
            else:
                self.get_logger().error("Failed to verify saved image")

            # If in debug mode, stop here
            if self.debug_mode:
                self.get_logger().info("Debug mode: Image saved, skipping API call")
                self.process_next_frame = False
                self.done_event.set()
                return

            # Send to OpenAI API
            response_text = self.process_with_openai(temp_filename)

            # Print response
            self.get_logger().info("\nOpenAI API Response:")
            print("\n" + "="*50)
            print(response_text)
            print("="*50 + "\n")

            # Set the flag to stop processing frames
            self.process_next_frame = False

            # Signal that we're done
            self.done_event.set()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            self.done_event.set()

    # Rest of the code remains the same...

    def process_with_openai(self, image_path):
        """Process the image with OpenAI Vision API"""
        self.get_logger().info("Sending image to OpenAI API...")

        # Read and encode the image
        with open(image_path, "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode('utf-8')

        completion = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        { "type": "text", "text": "what's in this image?" },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}",
                                "detail": "low",
                            },
                        },
                    ],
                }
            ],
        )

        return completion.choices[0].message.content

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraProcessor()

    try:
        # Process until the image is received and processed
        while not node.done_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()