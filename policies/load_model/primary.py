# 3rd party imports
from tf_keras import backend as K
from keras.optimizers import Adam
from keras.models import Sequential
# custom imports
from qenv.utils.train_utils import *
from qenv.core import QCAR_CONTROL_PROTOCAL
from qenv.core import NoImageException


class KerasModelPolicy:
    """
    A class responsible for managing the policy of a TensorFlow model.

    This class handles the loading and execution of a pre-trained model,
    specifically designed for controlling an autonomous vehicle's steering
    angle.

    Attributes:
        model_path (str): The file path to the model's weights.
        state (dict): A dictionary representing the current state of the
            vehicle control protocol.
        image (np.ndarray): The latest image from the vehicle's camera, used as
            input for the model.
        model_angle (Sequential): The TensorFlow/Keras model that predicts the
            steering angle.

    Methods:
        setup(default_throttle=float): Initializes the model, loads its weights,
            and sets the default throttle.
        execute(image: np.ndarray): Processes an input image and updates the vehicle's
        steering state based on the model's prediction.
    """

    def __init__(self, model_path: str) -> None:
        """
        Initialize the KerasModelPolicy object.

        Args:
            model_path (str): The path to the model's weights file.

        Attributes:
            model_path (str): Stores the path to the model's weights.
            state (dict): Holds the vehicle control protocol state.
            image (np.ndarray): Placeholder for the latest camera image.
            model_angle (Sequential): The Keras model for predicting steering angles.
        """
        self.modle_path = model_path
        self.state: dict = QCAR_CONTROL_PROTOCAL
        self.image: np.ndarray = None
        self.model_angle: Sequential = None

    def setup(self, default_throttle: float = 0.05) -> None:
        """
        Set up the model with the necessary configurations.

        This method initializes the model, loads its weights, and sets the default throttle
        and safety flag in the state.

        Args:
            default_throttle (float): The initial throttle value to be set for the vehicle,
            adjusted by a factor of 0.3.
        """
        optimizer: Adam = Adam(1e-4, weight_decay=0.0)
        self.model_angle = get_model(optimizer)
        self.model_angle.load_weights(self.modle_path)
        self.state['throttle'] = default_throttle / 0.3
        self.state['control_flags']['safe'] = False

    def execute(self, image: np.ndarray) -> None:
        """
        Execute the model prediction on the provided image.

        This method processes the input image, performs any necessary preprocessing, and
        then uses the model to predict the steering angle, which is stored in the state.

        Args:
            image (np.ndarray): The input image from the vehicle's camera.

        Raises:
            NoImageException: If the input image is None.
        """
        # handle exception
        if image is None:
            raise NoImageException("No image in this iteration")
        # process image
        image = read_img(image)
        image = preprocess(image)
        # get prediction
        model_input: np.ndarray = np.array(image)
        model_input = np.expand_dims(model_input, axis=0)
        model_input = K.cast_to_floatx(model_input)
        # write to the state
        self.state['steering'] = self.model_angle.predict(model_input)
