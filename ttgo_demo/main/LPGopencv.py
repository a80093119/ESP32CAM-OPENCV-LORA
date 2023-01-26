import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import copy


def plot_img(data):
    plt.figure()
    plt.imshow(data, cmap='gray', vmin=0, vmax=255)
    plt.axis('off')
    plt.show()


with open("C:/Users/user/Desktop/espcam.txt", "r") as f:
    str_img_test = f.read()
str_img_test = str_img_test.replace(" ", "")
str_img_test = str_img_test.replace(";", ",")
replace_n = str_img_test.replace("\n", "")
img_test = np.array(replace_n.split(","), dtype=np.uint8)
#img_test = np.array(str_img_test[:-1].split(","), dtype=np.uint8)
gray = img_test.reshape(240, 320)
plot_img(gray)

kernel_size = 3
blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
plot_img(blur_gray)

low_threshold = 50
high_threshold = 150
masked_edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
plot_img(masked_edges)

# rho = 1
# theta = np.pi/180
# threshold = 3
# min_line_length = 10
# max_line_gap = 1

# line_image = np.copy(gray)*0  # creating a blank to draw lines on

# lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
#                         min_line_length, max_line_gap)

# for line in lines:
#     for x1, y1, x2, y2 in line:
#         cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)

# #color_edges = np.dstack((masked_edges, masked_edges, masked_edges))
# combo = cv2.addWeighted(masked_edges, 0.8, line_image, 1, 0)
# plt.imshow(combo)

# cv.HoughCircles函数设置参数


image = np.copy(masked_edges)
circles = cv2.HoughCircles(
    image, cv2.HOUGH_GRADIENT, 1, 30, param1=80, param2=30, minRadius=30, maxRadius=50)
circles = np.uint16(np.around(circles))
print(circles)
for i in circles[0, :]:
    cv2.circle(image, (i[0], i[1]), i[2], (255, 0, 0), 2)
    cv2.circle(image, (i[0], i[1]), 2, (255, 0, 0), 2)
plt.imshow(image)


def dist_2_pt(x1, y1, x2, y2):
    return np.sqrt(pow(x1-x2, 2)+pow(y1-y2, 2))
    
circle_img = copy.deepcopy(gray[i[1]-int(0.4*i[2]):i[1]+int(0.4*i[2]),
         i[0]-int(0.4*i[2]):i[0]+int(0.4*i[2])])
for (y,x), value in np.ndenumerate(circle_img): 

    if circle_img[y,x] > 153: #Good Pixel
        circle_img[y,x]=255
    elif circle_img[y,x] < 154: #Bad Pixel
        circle_img[y,x]=0

plot_img(circle_img)
plot_img(gray)

x = circles[0][0, 0]
y = circles[0][0, 1]
r = circles[0][0, 2]
line_keep = []
for line in lines:
    for x1, y1, x2, y2 in line:
        dist_diff1 = dist_2_pt(x1, y1, x, y)
        dist_diff2 = dist_2_pt(x2, y2, x, y)
        if (dist_diff1 < 0.8*r) & (dist_diff2 < 0.8*r):
            if (min(x1, x2)-5 <= x <= max(x1, x2)+5) & (min(y1, y2)-5 <= y <= max(y1, y2)+5):
                line_keep.append(line)

line_image_filter = np.copy(gray)*0  # creating a blank to draw lines on
for line in line_keep:
    for x1, y1, x2, y2 in line:
        cv2.line(line_image_filter, (x1, y1), (x2, y2), (255, 0, 0), 3)
combo = cv2.addWeighted(image, 0.8, line_image_filter, 1, 0)
plt.imshow(combo)

l = line_keep[0][0]
cc = circles[0][0]
true_theta = 0
delta_x = abs(l[0] - cc[0])
delta_y = abs(l[1] - cc[1])
if (((l[0] - cc[0]) < 0) & ((l[1] - cc[1]) > 0)):
    true_theta = np.pi / 2 - math.atan(delta_y / delta_x)

elif (((l[0] - cc[0]) < 0) & ((l[1] - cc[1]) < 0)):
    true_theta = np.pi / 2 + math.atan(delta_y / delta_x)

elif (((l[0] - cc[0]) > 0) & ((l[1] - cc[1]) < 0)):
    true_theta = 3 * np.pi / 2 - math.atan(delta_y / delta_x)

elif (((l[0] - cc[0]) > 0) & ((l[1] - cc[1]) > 0)):
    true_theta = 3 * np.pi / 2 + math.atan(delta_y / delta_x)

print(true_theta / np.pi * 180)
