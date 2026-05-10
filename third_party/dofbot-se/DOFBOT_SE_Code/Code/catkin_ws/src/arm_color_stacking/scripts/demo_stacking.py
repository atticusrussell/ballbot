import cv2 as cv
from color_stacking import color_stacking
color_hsv = {"red": ((2, 100, 60), (11, 255, 200)),
             "green": ((55, 80, 20), (78, 255, 86)),
             "blue": ((110, 100, 30), (125, 255, 200)),
             "yellow": ((26, 100, 100), (32, 232, 230))}

if __name__ == '__main__':
    color_stacking = color_stacking()
    capture = cv.VideoCapture(0)
    while capture.isOpened():
        _, image = capture.read()
        image, pos = color_stacking.select_color(image, color_hsv,"3101")
        color_stacking.stacking_grap(pos)
        cv.imshow("image", image)
        cv.waitKey(10)
    cv.destroyAllWindows()
