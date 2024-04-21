# other imports
import os
import time
import math
# 3rd party imports
import cv2
import numpy as np
from typing import List
# quanser imports
from hal.utilities.image_processing import ImageProcessing
from pal.utilities.math import Filter
# custom imports
from qenv.core.protocols import QCAR_CONTROL_PROTOCAL


class NoContourException(Exception):
    """
    Exception raised when no contours are found in an image.

    Attributes:
        *args (object): Variable length argument list to be passed to the Exception base class.
    """

    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class EdgeFollowing:
    """
    EdgeFollowing class for autonomous vehicle lane detection and following.

    This class implements image processing and control strategies to detect lane edges and
    compute steering angles for autonomous vehicle navigation.

    Attributes:
        default_throttle (float): Default throttle value for vehicle speed control.
        image_width (int): The width of the image frame from the vehicle's camera.
        image_height (int): The height of the image frame from the vehicle's camera.
        slope_offset (float): Offset value for the slope calculation.
        intersect_offest (float): Offset value for the intercept calculation.
        state (dict): Dictionary to store the current state of the vehicle control.
        pre_cross_err (float): Previous cross-track error for PID controller.
        integral_error (float): Integral error for PID controller.
        cross_err (float): Current cross-track error for PID controller.
        previous_derivative_term (float): Previous derivative term for PID controller.
        crops (dict): Dictionary defining the crop dimensions for image preprocessing.
        k_p (float): Proportional gain for PID controller.
        k_i (float): Integral gain for PID controller.
        k_d (float): Derivative gain for PID controller.
        steering_filter (Filter): Low-pass filter for smoothing the steering signal.
        start (float): Timestamp for the start of the control loop.

    Methods:
        preprocess(raw_image: np.ndarray) -> np.ndarray:
            Preprocesses the raw image from the camera for lane detection.

        get_houghline_image(grey_image: np.ndarray) -> np.ndarray:
            Applies Hough Line Transform to detect lines in the image.

        get_contours(grey_image: np.ndarray) -> np.ndarray:
            Detects contours in the image which may represent lane boundaries.

        find_lane_edge(grey_image: np.ndarray, contours: np.ndarray) -> np.ndarray:
            Identifies the lane edge from the contours and creates a binary image.

        visual_steering_pid(input: tuple) -> float:
            Computes the steering angle based on the detected lane edge using a PID controller.

        execute(raw_image: np.ndarray) -> None:
            Executes the image processing and control strategy to navigate the vehicle.
    """

    def __init__(self, default_throttle: float = 0.08, image_width: int = 820, image_height: int = 410) -> None:
        # offsets on the image
        self.slope_offset: float = -0.7685840357368919
        self.intersect_offest: float = 526.6626842439156
        # init control protocol
        self.state = QCAR_CONTROL_PROTOCAL
        self.state['control_flags']['safe'] = False
        self.default_throttle: float = default_throttle
        # pid control variables
        self.pre_cross_err: float = 0.0
        self.integral_error: float = 0.0
        self.cross_err: float = 0.0
        self.previous_derivative_term: float = 0.0
        self.image_width = image_width
        self.image_height = image_height
        self.crops: dict = {"up": 270, "left": 0, "right": 0, "bottom": 0}
        self.k_p: float = -0.8
        self.k_i: float = -0.000
        self.k_d: float = -0.06
        self.steering_filter = Filter().low_pass_first_order_variable(25, 0.033)
        next(self.steering_filter)
        # start time
        self.start = time.time()

    def preprocess(self, raw_image: np.ndarray) -> np.ndarray:
        if raw_image is None:
            return None

        # crop the image
        crop_image: np.ndarray = raw_image[self.crops["up"]:, :]
        # convert to grey image
        grey_image: np.ndarray = cv2.cvtColor(crop_image, cv2.COLOR_BGR2GRAY)

        return grey_image

    def get_houghline_image(self, grey_image: np.ndarray) -> np.ndarray:
        if grey_image is None:
            return None

        # thresholds
        max_angle: float = 180
        min_angle: float = 100
        # get lines in the image
        edges: np.ndarray = cv2.Canny(grey_image, 50, 150, apertureSize=3)
        lines: np.ndarray = cv2.HoughLines(edges, 1, np.pi/180, 110)
        # return grey image if no lines
        if lines is None:
            return grey_image

        # traverse and draw
        for line in lines:
            rho: float = line[0][0]
            theta: float = line[0][1]
            angle: float = theta * (180 / np.pi)
            if min_angle <= angle <= max_angle:
                a: float = np.cos(theta)
                b: float = np.sin(theta)
                x0: float = a * rho
                y0: float = b * rho
                x1: int = int(x0 + 1000 * (-b))
                y1: int = int(y0 + 1000 * (a))
                x2: int = int(x0 - 1000 * (-b))
                y2: int = int(y0 - 1000 * (a))
                # draw lines on the image
                cv2.line(grey_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        return grey_image

    def get_contours(self, grey_image: np.ndarray) -> np.ndarray:
        if grey_image is None:
            return None
        # apply gaussian blur
        blurred = cv2.GaussianBlur(grey_image, (9, 9), 0)
        blurred = ImageProcessing.image_filtering_open(blurred)

        # Apply thresholding to separate road and off-road areas
        thresh: np.ndarray = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
        contours: List[np.ndarray] = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        return contours

    def find_lane_edge(self, grey_image: np.ndarray, contours: np.ndarray) -> np.ndarray:
        if grey_image is None or contours is None or len(contours) == 0:
            raise NoContourException("Contour not found")

        largest_contour: np.ndarray = max(contours, key=cv2.contourArea)
        # cv2.drawContours(grey_image, [largest_contour], -1, (0, 255, 0), 3)
        cv2.imshow("Grey", grey_image)
        hull: np.ndarray = cv2.convexHull(largest_contour)
        mask: np.ndarray = np.zeros_like(grey_image)
        cv2.fillPoly(mask, [hull], (255, 255, 255))
        # Calculate the difference between adjacent pixels
        diff: np.ndarray = cv2.Sobel(mask, cv2.CV_64F, 1, 1, ksize=15)
        cv2.imshow("Mask", mask)
        # Threshold the difference image to find the edge
        edge: np.ndarray = cv2.threshold(diff, 150, 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("Edge", edge)

        return edge

    def visual_steering_pid(self, input: tuple) -> float:
        # extract from tuple
        slope: float = input[0]
        intercept: float = input[1]
        # error state
        if slope == 0.3419:
            return 0.0

        # steering from slope and intercept
        if abs(slope) < 0.2 and abs(intercept) < 100:
            slope = self.slope_offset
            intercept = self.intersect_offest
        # PID controller design
        self.cross_err = (intercept / -slope) - (self.intersect_offest / -self.slope_offset)
        self.cross_err = self.cross_err / self.image_width
        # time interval
        dt: float = time.time() - self.start
        self.integral_error += dt * self.cross_err
        self.derivetive_error = (self.cross_err - self.pre_cross_err) / dt
        # calcuate steering angle
        raw_steering: float = self.k_p * self.cross_err + self.k_i * self.integral_error + self.k_d * self.derivetive_error
        steering: float = self.steering_filter.send((np.clip(raw_steering, -0.5, 0.5), dt))

        return steering

    def execute(self, raw_image: np.ndarray) -> None:
        grey_image: np.ndarray = self.preprocess(raw_image = raw_image)
        hough_image: np.ndarray = self.get_houghline_image(grey_image = grey_image)
        contours: np.ndarray = self.get_contours(grey_image = hough_image)
        binary_image: np.ndarray = self.find_lane_edge(grey_image = hough_image, contours = contours)
        result: tuple = ImageProcessing.find_slope_intercept_from_binary(binary=binary_image)
        self.state['steering'] = self.visual_steering_pid(result) / 0.5
        self.state['throttle'] = self.default_throttle / 0.3
        self.start = time.time()
        # return self.state