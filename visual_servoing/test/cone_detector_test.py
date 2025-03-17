#!/usr/bin/env python

import unittest
import numpy as np
import cv2
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vs_msgs.msg import ConeLocationPixel
from cv_bridge import CvBridge

from visual_servoing.cone_detector import ConeDetector

class TestConeDetector(unittest.TestCase):
    
    def setUp(self):
        # Initialize ROS
        rclpy.init()
        
        # Create the node
        self.node = ConeDetector()
        
        # Create a CV bridge for testing
        self.bridge = CvBridge()
        
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_initialization(self):
        """Test that the node initializes correctly with proper publishers and subscribers"""
        # Check publishers
        self.assertIsNotNone(self.node.cone_pub)
        self.assertEqual(self.node.cone_pub.topic_name, "/relative_cone_px")
        
        self.assertIsNotNone(self.node.debug_pub)
        self.assertEqual(self.node.debug_pub.topic_name, "/cone_debug_img")
        
        # Check subscriber
        self.assertIsNotNone(self.node.image_sub)
        self.assertEqual(self.node.image_sub.topic_name, "/zed/zed_node/rgb/image_rect_color")
        
        # Check other initialization values
        self.assertFalse(self.node.LineFollower)
        self.assertIsNotNone(self.node.bridge)
    
    @patch('visual_servoing.cone_detector.cd_color_segmentation')
    def test_image_callback(self, mock_color_segmentation):
        """Test the image callback function with a mock image"""
        # Create a test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add an orange cone-like object
        cv2.rectangle(test_image, (300, 300), (340, 400), (0, 165, 255), -1)
        
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
        
        # Mock the color segmentation function to return a bounding box
        mock_bbox = [(310, 320), (330, 400)]
        mock_color_segmentation.return_value = (mock_bbox, 1)
        
        # Create a mock publisher to capture published messages
        original_pub = self.node.cone_pub
        mock_pub = MagicMock()
        self.node.cone_pub = mock_pub
        
        original_debug_pub = self.node.debug_pub
        mock_debug_pub = MagicMock()
        self.node.debug_pub = mock_debug_pub
        
        # Call the callback
        self.node.image_callback(image_msg)
        
        # Verify color segmentation was called with the right parameters
        mock_color_segmentation.assert_called_once()
        args, _ = mock_color_segmentation.call_args
        self.assertEqual(args[1], None)
        np.testing.assert_array_equal(args[0], test_image)
        
        # Verify the message was published with correct values
        mock_pub.publish.assert_called_once()
        published_msg = mock_pub.publish.call_args[0][0]
        self.assertIsInstance(published_msg, ConeLocationPixel)
        
        # Check the calculated pixel coordinates
        expected_x = float(abs(mock_bbox[1][0] + mock_bbox[0][0]) / 2)  # 320.0
        expected_y = float(mock_bbox[1][1])  # 400.0
        
        self.assertEqual(published_msg.u, expected_x)
        self.assertEqual(published_msg.v, expected_y)
        
        # Verify debug image was published
        mock_debug_pub.publish.assert_called_once()
        
        # Restore original publishers
        self.node.cone_pub = original_pub
        self.node.debug_pub = original_debug_pub
    
    @patch('visual_servoing.cone_detector.cd_color_segmentation')
    def test_empty_detection(self, mock_color_segmentation):
        """Test behavior when no cone is detected"""
        # Create a test image with no cone
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
        
        # Mock the color segmentation function to return empty bounding box
        mock_color_segmentation.return_value = ([(0, 0), (0, 0)], 0)
        
        # Create a mock publisher to capture published messages
        original_pub = self.node.cone_pub
        mock_pub = MagicMock()
        self.node.cone_pub = mock_pub
        
        # Call the callback
        self.node.image_callback(image_msg)
        
        # Verify the message was published with correct values
        mock_pub.publish.assert_called_once()
        published_msg = mock_pub.publish.call_args[0][0]
        
        # Check the calculated pixel coordinates (should be 0,0)
        self.assertEqual(published_msg.u, 0.0)
        self.assertEqual(published_msg.v, 0.0)
        
        # Restore original publisher
        self.node.cone_pub = original_pub

if __name__ == '__main__':
    unittest.main()