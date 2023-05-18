""" Calculate variance of the rendered results
    @author: Qianyue HE
    @date: 2023-5-18
"""

import os
import json
import tqdm
import numpy as np
import configargparse
import matplotlib.pyplot as plt

from pathlib import Path
from exr_read import read_exr
from typing import Tuple, List

def get_options():
    # IO parameters
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file=True, help='Config file path')
    parser.add_argument("--gt",                 required = True, help = "Ground truth file path", type = str)
    parser.add_argument("-i", "--input_folder", required = True, help = "Input file path", type = str)
    parser.add_argument("-j", "--json_input",   required = True, help = "Json input file path (ROI selection)", type = str)
    parser.add_argument("-q", "--quantile",     default = 0., help = "Normalize the output picture with its <x> quantile value", type = float)
    parser.add_argument("-o", "--json_output",  default = "", help = "Json output file path (Variance record)", type = str)
    return parser.parse_args()

def get_selection(path: str):
    with open(path, 'r', encoding = 'utf-8') as file:
        rects = json.load(file)["rects"]
    result = []
    for rect in rects:
        result.append((tuple(rect["p1"]), tuple(rect["p2"])))
    return result

def outlier_rejection_mean(patch: np.ndarray, iter = 2):
    """ Reject firefly induced mean shifting
    """
    mean = patch.mean(axis = (0, 1))
    mask = np.ones(patch.shape[:-1], dtype = bool)
    mask &= patch.max(axis = -1) < 255                  # saturated pixel can not be used in calculation
    for _ in range(iter):
        diff = patch - mean
        diff_norm = np.linalg.norm(diff, axis = -1)
        max_norm = diff_norm.max()
        mask &= diff_norm < (0.9 * max_norm)
        mean = patch[mask, :].mean(axis = 0)
    return mean

def rescale_patches(patch: np.ndarray, patch_gt: np.ndarray):
    """ Quantile normalization might introduce scaling bias
    """
    patch_mean    = outlier_rejection_mean(patch, 1)
    patch_gt_mask = patch_gt.max(axis = -1) < 255 
    patch_gt_mean = patch_gt[patch_gt_mask, :].mean(axis = (0, 1))
    patch = patch / patch_mean * patch_gt_mean
    return patch

def variance_analysis(input_list: List[np.ndarray], gt: np.ndarray, roi: List[Tuple[Tuple[int, int], Tuple[int, int]]]):
    all_mse = []
    result_mse = []
    for i, ((sx, sy), (ex, ey)) in enumerate(roi):
        gt_patch = gt[sy:ey, sx:ex, :].copy()
        mses = []
        for img in input_list:
            patch = img[sy:ey, sx:ex, :].copy()
            # patch = rescale_patches(patch, gt_patch)
            mse = (patch - gt_patch) ** 2
            mses.append(mse.mean())
        final_mse = np.mean(mses)
        print(f"For ROI {i + 1}, variance = {final_mse:.6f}")
        all_mse.append(mses)
        result_mse.append(final_mse)
    return all_mse, result_mse

def get_input_images(input_path:str, quantile: float) -> List[np.ndarray]:
    image_names = os.listdir(input_path)
    image_names = list(filter(lambda name: name.endswith(".exr"), image_names))
    images = []
    for name in image_names:
        path = os.path.join(input_path, name)
        image = read_exr(path, quantile).astype(float)
        images.append(image)
    return images

if __name__ == "__main__":
    opts = get_options()
    roi_selection = get_selection(opts.json_input)
    gt_image      = read_exr(opts.gt, opts.quantile).astype(float)
    var_images    = get_input_images(opts.input_folder, opts.quantile) 
    all_mse, result_mse = variance_analysis(var_images, gt_image, roi_selection)

    if opts.json_output:
        out_path = os.path.join(opts.input_folder, opts.json_output)
        with open(out_path, 'w', encoding = 'utf-8') as file:
            json_file = {"name": Path(opts.input_folder).stem, "all_mse": all_mse, "mse": result_mse}
            json.dump(json_file, file, indent = 4)
