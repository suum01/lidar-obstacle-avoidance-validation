import Function_Library as FL
# import rplidar as RPLidar
import numpy as np
import cv2
import math
import time

# arduino
arduino_port = 'COM13'
ser = FL.libARDUINO()
comm = ser.init(arduino_port, 230400)

ultra = [0, 0, 0, 0, 0, 0]
Speed, Angle = 20, 0
sensor_flag = 2

start_flag = 0
cross_flag = False

WIDTH, HEIGHT = 800, 448

tl = [100, 300]
tr = [700, 300]
bl = [800, 448]
br = [0, 448]
corner_points_array = np.float32([tl, tr, br, bl])
img_tl = [0, 0]
img_tr = [WIDTH, 0]
img_bl = [WIDTH, HEIGHT]
img_br = [0, HEIGHT]
img_params = np.float32([img_tl, img_tr, img_br, img_bl])
matrix = cv2.getPerspectiveTransform(corner_points_array, img_params)

sensitivity = 100
lower_white = np.array([0, 0, 255 - sensitivity])
upper_white = np.array([255, sensitivity, 255])
lower_red1 = np.array([0, 150, 150])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 150, 150])
upper_red2 = np.array([180, 255, 255])

x_448 = [765 for i in range(10)]
principle_m_right = [100 for i in range(10)]

debug = True
corner_flag = 0

# Obstacle driving variable
first_scanning = 0
second_scanning = 0
Obstacle_start_flag = 0  # Flag to prevent re-entering obstacle avoidance mode
Obstacle_end_flag = 0

first_obstacle_flag = 0
first_recog_flag = 0
first_recog2_flag = 0
first_recog3_flag = 0
first_recog4_flag = 0
first_obstacle_endflag = 0

second_obstacle_flag = 0
second_recong_flag = 0
second_recong2_flag = 0
second_recong3_flag = 0
second_obstacle_endflag = 0

count = 0
count2 = 0
count3 = 0

e = 1


def drive(user_speed, user_angle, sen_flag):
    string = "%d,%d,%d\n" % (user_speed, user_angle, sen_flag)
    to_arduino = bytes(string, 'utf-8')
    print('transmit', to_arduino)
    comm.write(to_arduino)


def value_receive():
    receive_string = str(comm.readline())
    receive_string = receive_string[2:len(receive_string) - 5]
    ultra_val = receive_string.split(",")

    for i in range(0, 6):
        ultra_val[i] = int(ultra_val[i])

    return ultra_val


drive(0, 0, 2)
ultra_val = value_receive()

# Camera variable
line_cam = cv2.VideoCapture(cv2.CAP_DSHOW)
traffic_cam = cv2.VideoCapture(cv2.CAP_DSHOW + 2)

line_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
line_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
traffic_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
traffic_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def Obstacle_race():
    global first_scanning, second_scanning, Obstacle_start_flag, Obstacle_end_flag, first_obstacle_flag, first_recog_flag, first_recog2_flag, first_recog3_flag, first_recog4_flag, \
        first_obstacle_endflag, second_obstacle_flag, second_recong_flag, second_recong2_flag, second_recong3_flag, second_obstacle_endflag, count, count2, count3
    global line_detect_flag
    global x_448, principle_m_right

    env = FL.libLIDAR('COM6')
    env.init()

    if Obstacle_start_flag == 0 and Obstacle_end_flag == 0 and second_obstacle_endflag == 0 and first_scanning == 0:
        speed = 40
        angle = 0
        for scan in env.scanning():
            if line_detect_flag == 1:
                ret1, line_img = line_cam.read()
                cv2.imshow(line_img)
#                cv2.imshow(traffic_cam)
                bird_img = cv2.warpPerspective(line_img, matrix, (WIDTH, HEIGHT))
                copy2_bird_img = np.copy(bird_img)

                gray_line_img = cv2.cvtColor(bird_img, cv2.COLOR_BGR2GRAY)
                blur_gray_line = cv2.GaussianBlur(gray_line_img, (5, 5), 0)
                edge_line_img = cv2.Canny(np.uint8(blur_gray_line), 120, 160)

                Hough_lines = cv2.HoughLinesP(edge_line_img, 1, math.pi / 180, 60, 50, 10)
                if Hough_lines is None:
                    Angle = 12
                elif len(Hough_lines) < 3:
                    Angle = 10
                else:
                    right_lines = []
                    left_lines = []

                    for line in Hough_lines:
                        x1, y1, x2, y2 = line[0]
                        if x2 == x1:
                            slope = 400
                        else:
                            slope = float(y2 - y1) / float(x2 - x1)

                        if slope > 399:
                            cnt += 1

                    if cnt > (len(Hough_lines) * 0.5) or len(Hough_lines) > 40:
                        x_limit = 720
                    else:
                        x_limit = 500

                    for line in Hough_lines:
                        x1, y1, x2, y2 = line[0]

                        if x1 == x2:
                            slope = 1000
                        else:
                            slope = float(y2 - y1) / float(x2 - x1)

                        if (abs(slope) > 0.5) and (x1 > x_limit) and (x2 > x_limit):
                            right_lines.append(line[0])
                        if (abs(slope) > 0.5) and (x1 < 300) and (x2 < 300):
                            left_lines.append(line[0])

                    cnt = 0

                    for line in right_lines:
                        x1, y1, x2, y2 = line
                        cv2.line(copy2_bird_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

                    m_right, b_right = 0.0, 0.0
                    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
                    size = len(right_lines)
                    if size != 0:
                        for line in right_lines:
                            x1, y1, x2, y2 = line
                            x_sum += x1 + x2
                            y_sum += y1 + y2

                            if x2 != x1:
                                m_sum += float(y2 - y1) / float(x2 - x1)
                            else:
                                m_sum += 20
                                cnt += 1

                            if cnt > (len(right_lines) * 0.4):
                                vertical_flag = 1
                            else:
                                vertical_flag = 0

                        x_avg = x_sum / (size * 2)
                        y_avg = y_sum / (size * 2)
                        m_right = m_sum / size
                        b_right = y_avg - m_right * x_avg

                        x_448.pop(0)

                        if vertical_flag == 1:
                            m_right = 400
                            x_448.append(int(x_avg))
                            x_avg = int(x_avg)
                        #                    print(x_avg)
                        #                    principle_m_right.append(m_right)
                        #                    FL.cv2.line(copy2_bird_img, (x_avg, 0), (x_avg, HEIGHT), (255, 0, 127), 2)
                        else:
                            x1_ = int((0 - b_right) / m_right)
                            x2_ = int((448 - b_right) / m_right)
                            x_448.append(x_avg)
                        #                FL.cv2.line(copy2_bird_img, (x1_, 0), (x2_, HEIGHT), (255, 127, 0), 2)
                        #            FL.cv2.imshow('after principle', copy2_bird_img)

                        principle_m_right.pop(0)
                        principle_m_right.append(m_right)

                        x_mean = np.mean(x_448)
                        m_mean = np.mean(principle_m_right)
                        b_mean = 448 - m_mean * x_mean
                        x_0 = int((0 - b_mean) / m_mean)

                        FL.cv2.line(copy2_bird_img, (x_0, 0), (int(x_mean), HEIGHT), (255, 127, 0), 2)
                        FL.cv2.imshow('after principle', copy2_bird_img)

                        #                if m_mean < 0:
                        #                    gain_a = 0
                        #                else:
                        gain_a = 20

                        if x_mean > 760:
                            gain_m = 0.5
                        else:
                            gain_m = 0.1

                        angle = -(1 / m_mean) * gain_a - (770 - x_mean) * gain_m
                        angle = int(max(min(angle, 17), -17))
                #                print(m_mean, x_mean)
                #            else:
                #                for line in left_lines:

                if abs(angle) > 15:
                    speed = 75
                if abs(angle) > 10:
                    speed = 65
                elif abs(angle) > 5:
                    speed = 50
                elif abs(angle) > 2:
                    speed = 40
                else:
                    speed = 40

            cal_angle = (angle * 17) + 560

            drive(speed, cal_angle, 2)
            ultra_val = value_receive()
            print(ultra_val)

            scan = env.getAngleDistanceRange(scan, 250, 290, 500, 1300)  # 1000, 1500    # Front obstacle detection
            # scan = np.asarray(scan, dtype=int)  # convert float values to integers for easier comparison
            # print('start')

            # print(scan)
            # print(first_recog_flag)

            if Obstacle_start_flag == 0 and len(scan) > 0:  # If an obstacle is detected before obstacle driving starts
                scan = env.getAngleDistanceRange(scan, 250, 290, 500, 1300)
                # drive(20, 0, 0)
                # ultra_val= value_receive()
                speed = 40
                angle = 0
                Obstacle_start_flag = 1  # Start obstacle avoidance driving
                first_obstacle_flag = 1  # Start avoiding the first obstacle
                line_detect_flag = 0
                print('First Obstacle avoidance start')

            if first_obstacle_flag == 1 and first_recog_flag == 0 and first_recog2_flag == 0:
                scan = env.getAngleDistanceRange(scan, 250, 320, 500, 1500)
                if len(scan) > 0:  # While the first obstacle is detected in front of the vehicle
                    speed = 40
                    angle = -15
                    print('turn left')  # Steer left
                    # print(scan)
                elif len(scan) == 0:
                    first_recog_flag = 1
                # Use count to maintain steering for a certain period

            if first_obstacle_flag == 1 and first_recog_flag == 1 and first_recog2_flag == 0:
                # scan = env.getAngleDistanceRange(scan, 300, 355, 100, 1500)
                print('2')
                # if len(scan)==0:  # If no obstacle is detected in front during the first obstacle avoidance
                scan = env.getAngleDistanceRange(scan, 310, 355, 100, 1000)
                speed = 40
                angle = -15
                count += 1
                print(count)

                if count == 100:
                    speed = 40
                    angle = 0
                elif count > 115:
                    first_recog2_flag = 1
                    #count += 1
                print('scan')

            if first_obstacle_flag == 1 and first_recog_flag == 1 and first_recog2_flag == 1 and first_recog3_flag == 0:  # Detect the next obstacle after avoiding the first obstacle
                # drive(20, 15, 0)
                # ultra_val= value_receive()
                speed = 40
                angle = 13
                print('right')
                scan = env.getAngleDistanceRange(scan, 250, 290, 800, 1500)
                if first_recog3_flag == 0 and len(scan) > 0:
                    first_recog3_flag = 1

            if first_obstacle_flag == 1 and first_recog3_flag == 1 and second_obstacle_flag == 0:
                scan = env.getAngleDistanceRange(scan, 250, 290, 800, 1500)
                # drive(20, 15, 0)
                # ultra_val= value_receive()
                speed = 40
                angle = 13
                print('second obstacle')
                second_obstacle_flag = 1
                first_obstacle_endflag = 1

            if second_obstacle_flag == 1 and second_recong_flag == 0:
                # drive(20, 15, 0)
                # ultra_val= value_receive()
                speed = 40
                angle = 15
                print('second')
                scan = env.getAngleDistanceRange(scan, 155, 210, 650, 900)
                count2 += 1
                if count2 > 300:
                    print('on left')
                    second_recong_flag = 1

            if second_obstacle_flag == 1 and second_recong_flag == 1 and second_recong2_flag == 0:
                # drive(20, -15, 0)
                # ultra_val= value_receive()
                speed = 40
                angle = -12
                print('recong2')
                count3 += 1

            if count3 > 250:
                second_obstacle_endflag = 1
                print('end')
                env.stop()
                break
        print('obstacle out')


def main():
    global Speed, Angle
    global line_cam, traffic_cam
    global x_448, principle_m_right
    global cross_flag

    Obstacle_race()

    while True:
        ret1, line_img = line_cam.read()
        ret2, traffic_img = traffic_cam.read()

        bird_img = cv2.warpPerspective(line_img, matrix, (WIDTH, HEIGHT))
        cross_img = np.copy(bird_img)
        cross_hsv = cv2.cvtColor(cross_img, cv2.COLOR_BGR2HSV)
        cross_white = cv2.inRange(cross_hsv, lower_white, upper_white)
        white_bitwise = cv2.bitwise_and(cross_img, cross_img, mask=cross_white)
        gray_line_img = cv2.cvtColor(white_bitwise, cv2.COLOR_BGR2GRAY)
        n_all_pix = float(np.sum(gray_line_img >= 50))  # Total pixels in the ROI
        n_white_pix = float(np.sum(gray_line_img >= 200))  # White pixels in the ROI
        _percent = float((n_white_pix / n_all_pix) * 100)

        if _percent >= 95 and cross_flag is False:
            drive(0, 0, 2)
            sensor_val = value_receive()
            print('cross waiting')
            ex_percent = 20
            while True:
                ret2, traffic_img = traffic_cam.read()
                traffic_signal = np.copy(traffic_img)
                traffic_hsv = cv2.cvtColor(traffic_signal, cv2.COLOR_BGR2HSV)
                traffic_red1 = cv2.inRange(traffic_hsv, lower_red1, upper_red1)
                traffic_red2 = cv2.inRange(traffic_hsv, lower_red2, upper_red2)
                traffic_red = cv2.add(traffic_red1, traffic_red2)
                n_all_pix = float(np.sum(traffic_red >= 0))  # Total pixels in the ROI
                n_red_pix = float(np.sum(traffic_red >= 150))  # Red pixels in the ROI
                _percent = float((n_red_pix / n_all_pix) * 10000)
                if abs(_percent - ex_percent) > 15:
                    cnt += 1
                else:
                    ex_percent = _percent

                if cnt > 5:
                    break

        copy2_bird_img = np.copy(bird_img)

        gray_line_img = cv2.cvtColor(bird_img, cv2.COLOR_BGR2GRAY)
        blur_gray_line = cv2.GaussianBlur(gray_line_img, (5, 5), 0)
        edge_line_img = cv2.Canny(np.uint8(blur_gray_line), 120, 160)

        Hough_lines = cv2.HoughLinesP(edge_line_img, 1, math.pi / 180, 60, 50, 10)
        if Hough_lines is None:
            Angle = 12
        elif len(Hough_lines) < 3:
            Angle = 10
        else:
            right_lines = []
            left_lines = []

            for line in Hough_lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    slope = 400
                else:
                    slope = float(y2 - y1) / float(x2 - x1)

                if slope > 399:
                    cnt += 1

            if cnt > (len(Hough_lines) * 0.5) or len(Hough_lines) > 40:
                x_limit = 720
            else:
                x_limit = 500

            for line in Hough_lines:
                x1, y1, x2, y2 = line[0]

                if x1 == x2:
                    slope = 1000
                else:
                    slope = float(y2 - y1) / float(x2 - x1)

                if (abs(slope) > 0.5) and (x1 > x_limit) and (x2 > x_limit):
                    right_lines.append(line[0])
                if (abs(slope) > 0.5) and (x1 < 300) and (x2 < 300):
                    left_lines.append(line[0])

            cnt = 0

            for line in right_lines:
                x1, y1, x2, y2 = line
                cv2.line(copy2_bird_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

            m_right, b_right = 0.0, 0.0
            x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
            size = len(right_lines)
            if size != 0:
                for line in right_lines:
                    x1, y1, x2, y2 = line
                    x_sum += x1 + x2
                    y_sum += y1 + y2

                    if x2 != x1:
                        m_sum += float(y2 - y1) / float(x2 - x1)
                    else:
                        m_sum += 20
                        cnt += 1

                    if cnt > (len(right_lines) * 0.4):
                        vertical_flag = 1
                    else:
                        vertical_flag = 0

                x_avg = x_sum / (size * 2)
                y_avg = y_sum / (size * 2)
                m_right = m_sum / size
                b_right = y_avg - m_right * x_avg

                x_448.pop(0)

                if vertical_flag == 1:
                    m_right = 400
                    x_448.append(int(x_avg))
                    x_avg = int(x_avg)
                else:
                    x1_ = int((0 - b_right) / m_right)
                    x2_ = int((448 - b_right) / m_right)
                    x_448.append(x_avg)

                principle_m_right.pop(0)
                principle_m_right.append(m_right)

                x_mean = np.mean(x_448)
                m_mean = np.mean(principle_m_right)
                b_mean = 448 - m_mean * x_mean
                x_0 = int((0 - b_mean) / m_mean)

                FL.cv2.line(copy2_bird_img, (x_0, 0), (int(x_mean), HEIGHT), (255, 127, 0), 2)
                FL.cv2.imshow('after principle', copy2_bird_img)

                gain_a = 20

                if x_mean > 760:
                    gain_m = 0.5
                else:
                    gain_m = 0.1

                Angle = -(1 / m_mean) * gain_a - (770 - x_mean) * gain_m
                Angle = int(max(min(Angle, 17), -17))

        if abs(Angle) > 15:
            Speed = 75
        if abs(Angle) > 10:
            Speed = 90
        elif abs(Angle) > 5:
            Speed = 115
        elif abs(Angle) > 2:
            Speed = 130
        else:
            Speed = 160

        cal_angle = int((Angle * 20) + 600)

        drive(Speed, cal_angle, sensor_flag)
        sensor_val = value_receive()
        print(Speed, Angle)

        if FL.cv2.waitKey(10) & 0xFF == ord('q'):
            drive(0, 0, 2)
            break


if __name__ == "__main__":
    main()
