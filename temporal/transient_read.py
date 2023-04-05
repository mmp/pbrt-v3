import re
import os
import sys
import imageio
import numpy as np 
import OpenEXR, Imath
import matplotlib.pyplot as plt

from tqdm import tqdm
from copy import deepcopy
from opts import get_tdom_options
from shot_noise_removal import shot_peaks_detection, local_median_filter

colors = ("#DF7857", "#4E6E81", "#F99417")

def median_filter(x: np.ndarray, radius = 2):
    h, w, layers = x.shape
    for layer in range(layers):
        for ri in range(h):
            for ci in range(w):
                block = x[max(ri - radius, 0):min(ri + radius + 1, h), max(ci - radius, 0):min(ci + radius + 1, w), layer]
                value = x[ri, ci, layer]
                if np.isinf(value) or np.isnan(value):
                    median = np.median(block)
                    if np.isinf(median):
                        print("Warning: median value is inf:", block.ravel())
                        median = 0.
                    x[ri, ci, layer] = median  # median value
    return x

def get_transient_exr(path: str):
    inputFile = OpenEXR.InputFile(path)
    pixelType = Imath.PixelType(Imath.PixelType.HALF)
    dataWin = inputFile.header()['dataWindow']
    imgSize = (dataWin.max.y - dataWin.min.y + 1, dataWin.max.x - dataWin.min.x + 1)
    tmp = list(inputFile.header()['channels'].keys())
    if(len(tmp) != 3):
        prog = re.compile(r"\d+")
        channels = np.array(np.argsort([int(re.match(prog, x).group(0)) for x in tmp], -1, 'stable'))
        channels[0::3], channels[2::3] = deepcopy(channels[2::3]),deepcopy(channels[0::3])
        tmp = np.array(tmp)
        tmp = tmp[list(channels)]
    else:
        tmp = np.array(tmp)
        tmp[0], tmp[2] = tmp[2], tmp[0]

    transients = inputFile.channels(tmp, pixelType)
    transients = [np.reshape(np.frombuffer(transients[i], dtype=np.float16), imgSize) for i in range(len(transients))]
    transients = np.stack(transients, axis=2)
    h, w, _ = transients.shape
    transients = transients.reshape(h, w, -1, 3)
    if transients.shape[-2] == 1:
        transients = transients.squeeze(axis = -2)
    return transients.astype(np.float32)

if __name__ == "__main__":
    opts = get_tdom_options()
    
    all_transients = []
    for i in tqdm(range(opts.num_transient)):
        file_name = os.path.join(opts.input_path, f"{opts.input_name}_{i:04d}.exr")
        all_transients.append(get_transient_exr(file_name))
    all_transients = np.stack(all_transients, axis = 0)
    
    num_transient, h, w, _ = all_transients.shape
    
    window = h // 3
    curves = np.zeros((3, num_transient))
    for i in range(num_transient):
        for j in range(3):
            curves[j, i] = all_transients[i, j * window:(j + 1) * window, j * window:(j + 1) * window].mean()

    qnt = np.quantile(all_transients, opts.qnt)
    print(f"Quantial: {qnt}")
    output_path = os.path.join(opts.input_path, "outputs")
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    for i in tqdm(range(num_transient)):
        output_name = os.path.join(output_path, f"{i:04d}.png")
        imageio.imwrite(output_name, ((all_transients[i, ...] / qnt).clip(0, 1) * 255).astype(np.uint8))
            
    side_curves = (curves[0] + curves[2]) / 2.
    
    actual_time = (opts.time_length if opts.time_length > 0 else num_transient) / opts.sol
    xs = np.linspace(0, actual_time, num_transient)
    plt.title("PBRT simulation raw data curve")

    if opts.window_mode == 'diag_tri':
        curves /= curves.max(axis = 1, keepdims = True)
    elif opts.window_mode == 'diag_side_mean':
        curves /= side_curves.max()
        curves = np.float32([curves[0], curves[1]])
    else:       # whole
        curves = all_transients.mean(axis = (1, 2, 3))     # spatial average
    params = {'prominence': 0.00, 'threshold': 0.08}
    if curves.ndim == 1:
        curves = curves[None, :]
    for i in range(curves.shape[0]):
        peaks = shot_peaks_detection(curves[i], params)
        local_median_filter(curves[i], peaks)
        # plt.scatter(xs[peaks], curves[i, peaks], s = 40, facecolors = 'none', edgecolors = colors[(i + 1) % 3])   # visualizing shot noise peaks
        plt.scatter(xs, curves[i], s = 5, c = colors[i])
        plt.plot(xs, curves[i], label = f'window[{i+1}]', c = colors[i])
    
    plt.legend()
    plt.xlim((0, actual_time))
    plt.xlabel("Time (ns)")
    plt.grid(axis = 'both')
    plt.show()
    