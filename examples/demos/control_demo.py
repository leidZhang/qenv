# other imports
import os
# 3rd party imports
import cv2
# custom imports
from examples.policies.manual.keyboard import KeyboardController
from qenv.utils.qlab_utlis import connect_to_qlab
from qenv.qlab.qcar.control import VirtualControl
from qenv.qlab.qcar.sensor import VirtualCSICamera

def keyboard_control() -> None:
    try:
        connect_to_qlab()
        # module initialization
        my_csi: VirtualCSICamera = VirtualCSICamera(debug=True)
        running_gear: VirtualControl = VirtualControl()
        controller: KeyboardController = KeyboardController()
        # module setup
        controller.setup()
        # module execution
        while True:
            controller.execute()
            running_gear.execute(controller.state)
            my_csi.read_image()
            if cv2.waitKey(int(1)) & 0xFF == ord('m'):
                break
    except KeyboardInterrupt:
        os._exit(0)
