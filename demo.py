from examples.spawn_demo import spawn_demo
from examples.control_demo import keyboard_control
from examples.interprocess_demo import interprocess_demo
from examples.vision_lanefollowing import vision_lanefollowing

if __name__ == '__main__':
    spawn_demo(24)
    # keyboard_control()
    # interprocess_demo()
    vision_lanefollowing()