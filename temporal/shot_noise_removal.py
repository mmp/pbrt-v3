"""
    Shot noise removal for temporal response
"""

import numpy as np
from scipy.signal import find_peaks, peak_widths

def shot_peaks_detection(signal: np.ndarray, params: dict) -> np.ndarray:
    local_signal = signal / signal.max()
    prominence = params.get('prominence', 0.25)
    threshold = params.get('threshold', 0.1)
    peaks, _ = find_peaks(local_signal, prominence = prominence, threshold = threshold)
    return peaks

def local_median_filter(signal: np.ndarray, noise_peaks: np.ndarray, win_rad = 4):
    signal_length = signal.shape[-1]
    for peak in noise_peaks:
        min_t = max(0, peak - win_rad)
        max_t = min(signal_length, peak + win_rad + 1)
        if max_t - min_t < 3:
            raise ValueError("What's wrong with your signal?")
        local_part = signal[min_t:max_t]
        center_val = np.median(local_part)
        signal[peak] = center_val
    