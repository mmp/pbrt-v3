""" GUI: selecting image ROI for variance analysis
    @author: Qianyue HE
    @date: 2023-5-18
"""

import json
import cv2 as cv
import numpy as np
import configargparse
from exr_read import read_exr

img = None
screen = None
rect_p1 = None
rect_p2 = None
rects = []

def mouse_callback(event, x, y, flags, param):
    global rect_p1, rect_p2, img, screen
    if event == cv.EVENT_LBUTTONDOWN:
        rect_p1 = np.int32([x, y])
    elif event == cv.EVENT_LBUTTONUP:
        rects.append((rect_p1, rect_p2))
        rect_p1 = None
        rect_p2 = None
        print("One rectangle is added.")
    elif event == cv.EVENT_MOUSEMOVE:
        if rect_p1 is not None:
            rect_p2 = np.int32([x, y])

def get_options():
    # IO parameters
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file=True, help='Config file path')
    parser.add_argument("-i", "--input"   ,     required = True, help = "Input file path/name", type = str)
    parser.add_argument("-o", "--output_json" , default = "selection.json", help = "Output file path/name", type = str)
    parser.add_argument("-q", "--quantile",     default = 0., help = "Normalize the output picture with its <x> quantile value", type = float)
    parser.add_argument("-j", "--json_input",   default = "", help = "Visualize selected area", type = str)
    return parser.parse_args()

def area_selection(outpath: str):
    global img
    cv.namedWindow("selection")
    cv.setMouseCallback("selection", mouse_callback)
    while True:
        if rect_p1 is not None and rect_p2 is not None:
            min_pt = np.minimum(rect_p1, rect_p2)
            max_pt = np.maximum(rect_p1, rect_p2)
            screen = cv.rectangle(img.copy(), min_pt.tolist(), max_pt.tolist(), (0, 0, 255), thickness = 2)
        else:
            screen = img.copy()
        for i, (p1, p2) in enumerate(rects):
            min_pt = np.minimum(p1, p2)
            max_pt = np.maximum(p1, p2)
            screen = cv.rectangle(screen, min_pt, max_pt, color = (0, 255, 0), thickness = 2)
            screen = cv.putText(screen, f"{i+1}", (max_pt + 5).tolist(), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv.imshow("selection", screen)
        key = cv.waitKey(30)
        if key == 27:
            print("Returning without saving")
            break
        elif key == 80 or key == 112:
            if len(rects):
                print("Pop one rect.")
                rects.pop()
        elif key == 69 or key == 101:
            if len(rects):
                print(f"Saving selected area: {len(rects)} rect(s)")
                with open(outpath, 'w', encoding = 'utf-8') as file:
                    json_file = {"rects": []}
                    for i, (p1, p2) in enumerate(rects):
                        min_pt = np.minimum(p1, p2)
                        max_pt = np.maximum(p1, p2)
                        json_file["rects"].append({"p1": min_pt.tolist(), "p2": max_pt.tolist(), "id": i + 1})
                    json.dump(json_file, file, indent = 4)
                break
            else:
                print(f"To rect to save, please specify")
    cv.destroyAllWindows()

if __name__ == "__main__":
    opts = get_options()
    img = (read_exr(opts.input, opts.quantile) * 255).astype(np.uint8)
    img = img[..., ::-1]
    if opts.json_input:
        print("Visualize selected area")
        with open(opts.json_input, 'r', encoding = 'utf-8') as file:
            rects = json.load(file)["rects"]
        screen = img.copy()
        for rect in rects:
            screen = cv.rectangle(screen, rect["p1"], rect["p2"], color = (0, 255, 0), thickness = 2)
            screen = cv.putText(screen, f"{rect['id']}", (rect["p2"][0] + 5, rect["p2"][1] + 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.imshow("visualize", screen)
        cv.waitKey(0)
    else:
        area_selection(opts.output_json)



    