# 3rd party imports
import cv2
import numpy as np
import pandas as pd
from typing import Tuple, List, Optional
from pathlib import Path
import tensorflow as tf
from keras.models import Sequential
from keras.optimizers import Optimizer
from keras.layers import Lambda, Conv2D, Flatten, Dense, Activation, Dropout
import matplotlib.pyplot as plt

IMG_HT: int = 66
IMG_WIDTH: int = 200
IMG_CH: int = 3

def read_img_path(image_path: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)

def read_img(image: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def crop_and_resize(image: np.ndarray, width: int, height: int):
    image = image[40:-20, :, :]
    return cv2.resize(image, (width, height), cv2.INTER_AREA)

def preprocess(image: np.ndarray) -> np.ndarray:
    image = crop_and_resize(image, IMG_WIDTH, IMG_HT)
    return cv2.cvtColor(image, cv2.COLOR_RGB2YUV)

def read_csv_and_rename_cols(file_path: str, columns=None):
    path: Path = Path(file_path)
    if path.is_file() and path.exists():
        if columns:
            df = pd.read_csv(file_path, header=None)
            df = df.rename(columns=columns)
        else :
            df = pd.read_csv(file_path)
        return df

def get_relative_path(image_path: str) -> str:
    relative_path: str = './' + image_path.split('\\')[-2] + '/' + image_path.split('\\')[-1]
    return relative_path

def get_kernel(no: int) -> tf.Tensor:
    if no < 2 :
        return tf.constant(np.ones((3,3,1,1)), tf.float32)
    return tf.constant(np.ones((5,5,1,1)), tf.float32)

def get_stride(no: int) -> list:
    if no < 2 :
        return [1,1,1,1]
    return [1,2,2,1]

def reverse(lst: list) -> list:
    return [ele for ele in reversed(lst)]

def get_salient_feature_mask(ops: List[np.ndarray]) -> np.ndarray:
    i: int = 0
    upscaled_conv: np.ndarray = np.ones((1, 18))
    layer_ops: List[np.ndarray] = reverse(ops)
    for layer in layer_ops:
        avg_actvn: np.ndarray = np.mean(layer, axis=3).squeeze(axis=0)
        avg_actvn = avg_actvn * upscaled_conv
        output_shape: Tuple[int, int]
        if i == 4:
            output_shape = (IMG_HT, IMG_WIDTH)
        else:
            output_shape = (layer_ops[i+1].shape[1], layer_ops[i+1].shape[2])
        x: tf.Tensor = tf.constant(np.reshape(avg_actvn, (1, avg_actvn.shape[0], avg_actvn.shape[1], 1)), dtype=tf.float32)
        deconv: tf.Tensor = tf.nn.conv2d_transpose(x, get_kernel(i), (1, output_shape[0], output_shape[1], 1), get_stride(i), padding='VALID')
        with tf.Session() as session:
            res: np.ndarray = session.run(deconv)
        deconv_actn: np.ndarray = np.reshape(res, output_shape)
        upscaled_conv = deconv_actn
        i += 1
    mask: np.ndarray = (upscaled_conv - np.min(upscaled_conv)) / (np.max(upscaled_conv) - np.min(upscaled_conv))
    return mask

def get_model(optimizer: Optimizer, loss: str = 'mse') -> Sequential:
    model: Sequential = Sequential()

    model.add(Lambda(lambda x: x /127.5 - 1.0, input_shape = (IMG_HT, IMG_WIDTH, IMG_CH)))

    model.add(Conv2D(24, (5, 5), activation="elu", strides=(2, 2)))
    model.add(Conv2D(36, (5, 5), activation="elu", strides=(2, 2)))
    model.add(Conv2D(48, (5, 5), activation="elu", strides=(2, 2)))
    model.add(Conv2D(64, (3, 3), activation="elu", strides=(1, 1)))
    model.add(Conv2D(64, (3, 3), activation="elu", strides=(1, 1)))

    model.add(Flatten())
    model.add(Dropout(0.5))

    model.add(Dense(100))
    model.add(Activation('relu'))
    model.add(Dense(50))
    model.add(Activation('relu'))
    model.add(Dense(10))
    model.add(Dense(1))

    model.compile(loss=loss, optimizer=optimizer)

    return model

def display_multiple_images(
    img_list: List[np.ndarray],
    label_list: Optional[List[str]] = [],
    cmap: Optional[str] = '',
    is_yuv: bool = True
) -> None:
    assert(len(img_list) % 3 == 0)
    if len(label_list) > 0:
        assert(len(img_list) == len(label_list))

    cols: int = 3
    rows: int = int(len(img_list) / 3)
    fig = plt.figure(figsize=(16, 16))

    for i in range(1, cols * rows + 1):
        fig.add_subplot(rows, cols, i)
        img: np.ndarray = img_list[i - 1]
        if len(label_list) > 0:
            title: str = label_list[i - 1]
            plt.title(title)
        if is_yuv:
            img = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
        if cmap:
            plt.imshow(img, cmap=cmap)
        else:
            plt.imshow(img)
    plt.show()