import cv2 as cv
from color_identify import color_identify

if __name__ == '__main__':
    color_identify = color_identify()
    capture = cv.VideoCapture(0)
    color_hsv= {"red": ((0, 25, 90), (10, 255, 255)),
                "green": ((53, 36, 40), (80, 255, 255)),
                "blue": ((116, 80, 90), (130, 255, 255)),
                "yellow": ((25, 20, 55), (50, 255, 255))}
    while capture.isOpened():
        _, image = capture.read()
        image, pos = color_identify.select_color(image, color_hsv,"3101")
        color_identify.identify_grap(pos)
        cv.imshow("image", image)
        cv.waitKey(10)
    cv.destroyAllWindows()
