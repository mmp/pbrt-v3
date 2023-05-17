import configargparse
import numpy as np 
import OpenEXR, Imath
import matplotlib.pyplot as plt

def get_options():
    # IO parameters
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file=True, help='Config file path')
    parser.add_argument("-i", "--input"   , required = True, help = "Input file path/name", type = str)
    parser.add_argument("-o", "--output"  , default = "image.png", help = "Output file path/name", type = str)
    parser.add_argument("-q", "--quantile", default = 0., help = "Normalize the output picture with its <x> quantile value", type = float)
    return parser.parse_args()


if __name__ == '__main__':
    opts = get_options()
    inputFile = OpenEXR.InputFile(opts.input)
    pixelType = Imath.PixelType(Imath.PixelType.HALF)
    dataWin = inputFile.header()['dataWindow']
    imgSize = (dataWin.max.y - dataWin.min.y + 1, dataWin.max.x - dataWin.min.x + 1)
    tmp = list(inputFile.header()['channels'].keys())

    tmp = np.array(tmp)
    tmp[0], tmp[2] = tmp[2], tmp[0]

    channels = inputFile.channels(tmp, pixelType)
    images = [np.reshape(np.frombuffer(channels[i], dtype=np.float16), imgSize) for i in range(len(channels))]
    image = np.stack(images, axis=2)
    if opts.quantile > 0.1:
        image /= np.quantile(image, opts.quantile)
        image = np.clip(image, 0, 1)
    plt.imsave(opts.output, image)