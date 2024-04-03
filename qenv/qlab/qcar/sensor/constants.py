import numpy as np

CSI_CAMERA_SETTING: dict = {
    'focal_length': np.array([[157.9], [161.7]], dtype = np.float64) ,
    'principle_point': np.array([[168.5], [123.6]], dtype = np.float64),
    'position': np.array([[0], [0], [0.14]], dtype = np.float64),
    'orientation': np.array([[ 0, 0, 1], [ 1, 0, 0], [ 0, -1, 0]], dtype = np.float64),
    'frame_width': 820,
    'frame_height': 410,
    'frame_rate': 33.3
}

RGBD_CAMERA_SETTING: dict = {
    'mode': 'RGB, Depth',
    'frame_width_rgb': 640,
    'frame_height_rgb': 480,
    'frame_rate_rgb': 30.0,
    'frame_width_depth': 640,
    'frame_height_depth': 480,
    'frame_rate_depth': 15.0,
    'device_id': '0@tcpip://localhost:18965'
}