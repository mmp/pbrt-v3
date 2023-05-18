""" Read EXR image
    @author: Qianyue HE
    @date: 2023-5-17
"""

import configargparse
import numpy as np 
import OpenEXR, Imath
import matplotlib.pyplot as plt

__all__ = ["read_exr"]

def get_options():
    # IO parameters
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file=True, help='Config file path')
    parser.add_argument("-i", "--input"   , required = True, help = "Input file path/name", type = str)
    parser.add_argument("-o", "--output"  , default = "image.png", help = "Output file path/name", type = str)
    parser.add_argument("-q", "--quantile", default = 0., help = "Normalize the output picture with its <x> quantile value", type = float)
    return parser.parse_args()

def read_exr(input_path: str, quantile: float = 0.) -> np.ndarray:
    """ Reading from exr image and apply possible rescaling (by quantile)
    """
    inputFile = OpenEXR.InputFile(input_path)
    pixelType = Imath.PixelType(Imath.PixelType.HALF)
    dataWin = inputFile.header()['dataWindow']
    imgSize = (dataWin.max.y - dataWin.min.y + 1, dataWin.max.x - dataWin.min.x + 1)
    tmp = list(inputFile.header()['channels'].keys())

    tmp = np.array(tmp)
    tmp[0], tmp[2] = tmp[2], tmp[0]

    channels = inputFile.channels(tmp, pixelType)
    images = [np.reshape(np.frombuffer(channels[i], dtype=np.float16), imgSize) for i in range(len(channels))]
    image = np.stack(images, axis=2)
    if quantile > 0.1:
        image /= np.quantile(image, quantile)
        image = np.clip(image, 0, 1)
    return image

if __name__ == '__main__':
    opts = get_options()
    image = read_exr(opts.input, opts.quantile)
    plt.imsave(opts.output, image)