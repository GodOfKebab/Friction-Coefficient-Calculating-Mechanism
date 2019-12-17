from threading import Thread
from imutils.video import VideoStream
import cv2
import time
import imutils
import math
import argparse
import matplotlib.pyplot as plt
import numpy as np


parser = argparse.ArgumentParser(
    description='This program calculates either the static or kinetic friction coefficient between two surfaces.')
parser.add_argument('mode', type=str, default=None,
                    help='Chose mode. The mode can either be "static" or "kinetic"')

args = parser.parse_args()
mode = args.mode


class Vision(Thread):
    def __init__(self, system):

        super().__init__()

        self.camera = VideoStream(usePiCamera=True, resolution=(688, 528)).start()
        time.sleep(0.5)
        self.tracker = cv2.TrackerMOSSE_create()

        self.isTracking = None
        self.initBB = None
        self.frame = None

        self.initial_target_object_center = None
        self.initial_time = time.time()
        self.moving = False
        self.motion_detected = False
        self.speed = 0

        self.system = system
        self.is_running = True

        self.framesToShow = dict()
        self.isWindowShowEnabled = False
        self.key = "empty"

        self.coefficient_of_static_friction = 0.0
        self.coefficient_of_kinetic_friction = 0.0

    def run(self):
        while self.is_running:
            frame = self.camera.read()

            # Object tracking
            if self.isTracking:
                (success, box) = self.tracker.update(frame)

                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    final_target_object_center = (x + w / 2, y + h / 2)
                    self.speed = self.get_speed(final_target_object_center)

                    if self.speed > frame.shape[0] * frame.shape[1] / 100000:
                        self.motion_detected = True
                        self.system.start_recording()
                        self.moving = True
                    else:
                        self.moving = False

            # Arrange the screen
            frame = self.arrange_screen(frame)
            self.showFrame(frame)

        self.isWindowShowEnabled = False
        plt.close()

    # Arrange the general screen
    def arrange_screen(self, frame):
        # General Screen
        frame = cv2.rectangle(frame, (0, 490), (688, 528), (0, 0, 0), -1)
        frame = cv2.rectangle(frame, (510, 0), (688, 30), (0, 0, 0), -1)
        frame = cv2.putText(frame, "Angle:" + str(round(self.system.pot_angle, 1)), (10, 517), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 200), 2, cv2.LINE_AA)
        frame = cv2.putText(frame, "Distance:" + str(round(self.system.sonar, 2)), (520, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 200), 2, cv2.LINE_AA)

        # Custom Screen Settings
        if mode == "static":
            frame = self.arrange_screen_static(frame)
        elif mode == "kinetic":
            frame = self.arrange_screen_kinetic(frame)
        else:
            raise Exception("Wrong mode selected. Please Specify the mode as either 'static' or 'kinetic'")

        return frame

    # Specialize arrange_screen for static friction calculations
    def arrange_screen_static(self, frame):

        if self.moving:
            frame = cv2.putText(frame, "Moved!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 200), 2, cv2.LINE_AA)
            self.coefficient_of_static_friction = round(math.tan(math.pi * self.system.pot_angle / 180.0), 2)

        if self.motion_detected:
            frame = cv2.putText(frame, "coefficient of static friction:" + str(self.coefficient_of_static_friction),
                                (300, 517), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 200), 1, cv2.LINE_AA)

        return frame

    # Specialize arrange_screen for kinetic friction calculations
    def arrange_screen_kinetic(self, frame):

        if self.motion_detected:
            frame = cv2.putText(frame, "Started Measuring!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 200), 2,
                                cv2.LINE_AA)
            frame = cv2.putText(frame, "Press the button to stop measuring", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                (0, 0, 200), 2, cv2.LINE_AA)

        if self.moving:
            self.system.set_motor_speed(0)

        # End reading data
        if self.system.button_state and len(self.system.recorded_data) > 10:
            # Stop mechanism
            self.system.set_motor_speed(0)
            self.motion_detected = False

            # Process the recorded data
            data = self.system.end_recording_data()

            x = list()
            y = list()
            for i in data:
                x.append(i["time_stamp"])
                y.append(i["distance"] / 100.0)

            p = plotter()
            position_v_time = [x, y]
            position_v_time = p.trim_data(position_v_time)
            p.plot("Position", position_v_time)

            position_v_time_eq = p.plot_equation("Position", position_v_time)
            velocity_v_time_eq = p.take_derivative(position_v_time_eq)
            acceleration_v_time_eq = p.take_derivative(velocity_v_time_eq)

            _ = p.plot_equation("Velocity", position_v_time, eq=velocity_v_time_eq)
            _ = p.plot_equation("Acceleration", position_v_time, eq=acceleration_v_time_eq)

            print("\n\n*********************")
            print("Position vs. Time Graph's Equation is:")
            print(position_v_time_eq)
            print("\n*********************")
            print("Velocity vs. Time Graph's Equation is:", velocity_v_time_eq)
            print("*********************")
            print("Acceleration vs. Time Graph's Equation is:", acceleration_v_time_eq, "\n", "*********************")

            coefficient_of_static_friction = round(math.tan(math.pi * self.system.pot_angle / 180.0), 2) - float(acceleration_v_time_eq.c[0]) / (9.81 * round(math.cos(math.pi * self.system.pot_angle / 180.0), 2))

            print("Therefore the coefficient of kinetic friction is:{}".format(coefficient_of_static_friction))

            p.show()

        return frame

    # Multi-threaded window showing function
    def showFrame(self, frameToShow, windowName="Frame"):

        self.framesToShow[windowName] = frameToShow

        if not self.isWindowShowEnabled:
            self.isWindowShowEnabled = True
            Thread(target=self.__updateWindowFrame__, args=()).start()

    # Thread for updating the frame
    def __updateWindowFrame__(self):

        while self.isWindowShowEnabled:

            for name in self.framesToShow.copy():
                cv2.imshow(name, self.framesToShow[name])

            self.key = cv2.waitKey(30)

            if self.key == ord("s"):
                initBB = cv2.selectROI("Frame", self.framesToShow["Frame"], fromCenter=False, showCrosshair=True)
                self.tracker.init(self.framesToShow["Frame"], initBB)
                self.isTracking = True

            if self.key == ord('r'):
                self.motion_detected = False
                self.tracker = cv2.TrackerMOSSE_create()
                self.isTracking = False
                self.system.enabled_recording = False
                self.system.recorded_data = list()

        cv2.destroyAllWindows()

    # Calculate the velocity(pixel/seconds) of the selected object
    def get_speed(self, target_center):
        elapsed = time.time() - self.initial_time

        if self.initial_target_object_center is None:
            self.initial_target_object_center = target_center
            speed = 0
        else:
            displacement = ((target_center[0] - self.initial_target_object_center[0]) ** 2 +
                            (target_center[1] - self.initial_target_object_center[1]) ** 2) ** 0.5
            speed = displacement / elapsed

        self.initial_time = time.time()
        self.initial_target_object_center = target_center
        return speed


class plotter:
    def __init__(self):
        self.fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 8))
        plt.subplots_adjust(top=0.95, bottom=0.05)

        ax1.set_title("Position v. Time")
        ax1.set_xlabel("Time(s)")
        ax1.set_xlim(left=0.0)
        ax1.set_ylabel("Position(m)")
        ax1.set_ylim(bottom=0.0)
        ax1.grid(True)
        ax1.autoscale(True)

        ax2.set_title("Velocity v. Time")
        ax2.set_xlabel("Time(s)")
        ax2.set_xlim(left=0.0)
        ax2.set_ylabel("Velocity(m/s)")
        ax2.set_ylim(bottom=0.0)
        ax2.grid(True)
        ax2.autoscale(True)

        ax3.set_title("Acceleration v. Time")
        ax3.set_xlabel("Time(s)")
        ax3.set_xlim(left=0.0)
        ax3.set_ylabel("Acceleration(m/s^2)")
        ax3.set_ylim(bottom=0.0)
        ax3.grid(True)
        ax3.autoscale(True)

        self.ax = {
            "Position": ax1,
            "Velocity": ax2,
            "Acceleration": ax3
        }

    def take_derivative(self, eq):

        # second degree polynomial
        if len(eq.c) == 3:
            new_eq = eq.deriv()

        elif len(eq.c) == 2:
            new_eq = eq.deriv()

        else:
            raise Exception("Your equation must be either of 1st or 2nd degree")

        return new_eq

    def trim_data(self, data):
        x = data[0]
        y = data[1]

        new_x = list()
        new_y = list()

        for t in range(0, len(y)):
            x[t] = x[t] - (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9])/10.0
            y[t] = y[t] - (y[0] + y[1] + y[2] + y[3] + y[4] + y[5] + y[6] + y[7] + y[8] + y[9])/10.0

        for t, pos in enumerate(y):
            if pos < 0.35 and pos > 0.03:
                new_x.append(x[t])
                new_y.append(pos)



        return [new_x, new_y]


    def plot(self, graph_of, plot_data):
        self.ax[graph_of].plot(plot_data[0], plot_data[1], **{"marker": "o"})

    def plot_equation(self, graph_of, data_lists, eq=None):
        x = data_lists[0]
        y = data_lists[1]

        t = np.linspace(0, x[-1] + 0.1, y[-1] + 10)

        if graph_of == "Position":
            p_pos = np.poly1d(np.polyfit(x, y, 2))

        elif graph_of == "Velocity" or graph_of == "Acceleration":
            p_pos = eq

        else:
            raise Exception("You can only plot Position, Velocity or Acceleration")

        self.ax[graph_of].plot(x, y, 'o', t, p_pos(t), '-')

        return p_pos

    def show(self):
        plt.show()


from RpiMotorLib import rpi_dc_lib
from RPi import GPIO


class InclinedSurface(Thread):
    def __init__(self):
        super().__init__()
        self.motor = rpi_dc_lib.L298NMDc(19, 13, 26, 50)

        self.pot_angle = 0.0
        self.sonar = 0
        self.button_state = False
        self.is_running = True

        self.recorded_data = list()
        self.enabled_recording = False

        self.percent = 0

    def run(self):
        import serial  # Import Serial Library

        arduinoSerialData = serial.Serial('/dev/ttyS0', 57600)  # Create Serial port object called arduinoSerialData

        while self.is_running:
            try:
                my_data = arduinoSerialData.readline()
                str_my_data = str(my_data, encoding="utf-8").split("\r\n")[0]
                list_my_data = str_my_data.split(",")

                pot = int(list_my_data[0])
                self.pot_angle = -0.257 * pot + 219.0

                sonar = float(list_my_data[1])
                self.sonar = sonar if sonar < 40 else self.sonar

                self.button_state = int(list_my_data[2])

                if self.enabled_recording:
                    measurement = {"angle": self.pot_angle,
                                   "distance": self.sonar,
                                   "time_stamp": time.time()}

                    self.recorded_data.append(measurement)

                if self.percent > 0:
                    self.motor.backward(self.percent)
                elif self.percent < 0:
                    self.motor.forward(-self.percent)
                else:
                    self.motor.stop(0)

                if not (-7 < self.pot_angle < 60):
                    self.percent = 0

            except:
                pass

    def start_recording(self):
        self.enabled_recording = True

    def end_recording_data(self):
        self.enabled_recording = False

        initial_time = self.recorded_data[0]["time_stamp"]
        for index in range(len(self.recorded_data)):
            self.recorded_data[index]["time_stamp"] = self.recorded_data[index]["time_stamp"] - initial_time

        return self.recorded_data

    def set_motor_speed(self, percent):
        self.percent = percent

    def get_to_starting_point(self):

        if self.pot_angle > 0:
            while self.pot_angle > 0:
                self.set_motor_speed(-50)
                time.sleep(0.01)
        else:
            while self.pot_angle < 0:
                self.set_motor_speed(50)
                time.sleep(0.01)

        self.set_motor_speed(0)
        print("\n************\n")
        print("The Mechanism has been set to its default position. Ready to set motor speed")
        print("\n************\n")


system = InclinedSurface()
system.start()

vis = Vision(system)
vis.start()

system.get_to_starting_point()


while True:
    try:
        val = input("Set Motor:")
        try:
            if val == "default":
                system.get_to_starting_point()
            else:
                system.set_motor_speed(int(val))
        except:
            print("\nOnly decimal numbers allowed!\n")

    except KeyboardInterrupt:
        print("\nexiting\n")
        system.is_running = False
        vis.is_running = False
        system.set_motor_speed(0)
        time.sleep(0.5)
        exit()


