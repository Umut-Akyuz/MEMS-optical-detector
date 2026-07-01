from picamera2 import Picamera2
import cv2
import numpy as np
import math
import time
import csv
import pyvisa 

D = 68.0
PIXEL_PER_MM = 3.680 

ROI_X1, ROI_X2 = 240, 400

SETTLE_TIME = 1.0 
VOLTAGE_VPP = 1.0

freqs_part1 = np.arange(1.0, 40.0, 0.5)
freqs_part2 = np.arange(40.0, 55.0, 0.1)
freqs_part3 = np.arange(55.0, 300.5, 0.5)
sweep_frequencies = np.concatenate((freqs_part1, freqs_part2, freqs_part3))

print(">>> Searching for Keysight Generator...")
rm = pyvisa.ResourceManager('@py')
usb_devices = [dev for dev in rm.list_resources() if 'USB' in dev]

if not usb_devices:
    print("[ERROR] Keysight device not found via USB! Please check the cable.")
    exit()

try:
    inst = rm.open_resource(usb_devices[0])
    inst.timeout = 2000
    print(f">>> Connection Successful: {inst.query('*IDN?').strip()}")
    
    inst.write("*CLS")
    inst.write("SOUR:FUNC SIN")           
    inst.write(f"SOUR:VOLT {VOLTAGE_VPP}") 
    inst.write("SOUR:VOLT:OFFS 0")         
    inst.write("OUTP ON")                  
except Exception as e:
    print(f"[ERROR] Could not communicate with the device: {e}")
    exit()

timestamp = time.strftime("%H%M%S")
csv_filename = f"/home/pi/Desktop/tosa_resonance_direct_{timestamp}.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file, delimiter=';')
csv_writer.writerow(["Time", "Freq_Hz", "L_mm", "Angle_deg", "Threshold"])

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}, controls={"FrameRate": 120})
picam2.configure(config)
picam2.set_controls({"AeEnable": False, "ExposureTime": 10000, "AnalogueGain": 1.5})
picam2.start()

frame_counter = 0
last_time = time.time()
fps = 0
smoothed_L_mm = 0.0

freq_index = 0
current_freq = sweep_frequencies[freq_index]
current_threshold = 250
inst.write(f"SOUR:FREQ {current_freq}")
last_freq_update_time = time.time()

max_L_mm_in_step = 0.0
max_angle_in_step = 0.0

print(">>> SYSTEM READY! Smart autonomous sweep started.")
print(">>> Only the MAXIMUM value will be recorded for each frequency.")
print(">>> Press 'q' to cancel.")

try:
    while True:
        now_time = time.time()
        
        if (now_time - last_freq_update_time) >= SETTLE_TIME:
            
            str_freq = str(round(current_freq, 1)).replace('.', ',')
            str_L_mm = str(round(max_L_mm_in_step, 2)).replace('.', ',')
            str_angle = str(round(max_angle_in_step, 2)).replace('.', ',')
            
            csv_writer.writerow([time.strftime("%H:%M:%S"), str_freq, str_L_mm, str_angle, current_threshold])
            
            freq_index += 1
            if freq_index < len(sweep_frequencies):
                current_freq = sweep_frequencies[freq_index]
                inst.write(f"SOUR:FREQ {current_freq}")
                
                if 40.0 <= current_freq <= 55.0:
                    current_threshold = 210
                else:
                    current_threshold = 250
                    
                max_L_mm_in_step = 0.0
                max_angle_in_step = 0.0
                
                last_freq_update_time = now_time
                print(f"[{str_freq} Hz recorded] -> New Freq: {current_freq:.1f} Hz | Thresh: {current_threshold}")
            else:
                print("\n>>> SWEEP SUCCESSFULLY COMPLETED! <<<")
                break 
        
        frame = picam2.capture_array()
        frame_counter += 1
        frame = frame[:, ROI_X1:ROI_X2]
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        _, thresh = cv2.threshold(gray_blurred, int(current_threshold), 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tosa_angle = 0
        L_pixel = 0

        if contours:
            c = max(contours, key=cv2.contourArea)
            y_coords = c[:, 0, 1]
            x_coords = c[:, 0, 0]
            
            y_min = int(np.min(y_coords))
            y_max = int(np.max(y_coords))
            x_center = int(np.mean(x_coords))

            L_pixel = y_max - y_min

            if L_pixel > 15:
                L_mm_raw = L_pixel / PIXEL_PER_MM
                if smoothed_L_mm == 0:
                    smoothed_L_mm = L_mm_raw
                else:
                    smoothed_L_mm = (0.2 * L_mm_raw) + (0.8 * smoothed_L_mm)
                
                cv2.line(frame, (x_center, y_min), (x_center, y_max), (0, 255, 0), 2)
                cv2.circle(frame, (x_center, y_min), 3, (0, 0, 255), -1)
                cv2.circle(frame, (x_center, y_max), 3, (0, 0, 255), -1)

        L_mm = smoothed_L_mm

        if L_mm > 0:
            tosa_rad = 2 * math.atan(L_mm / (2 * D))
            tosa_angle = math.degrees(tosa_rad)

            if tosa_angle > max_angle_in_step:
                max_angle_in_step = tosa_angle
                max_L_mm_in_step = L_mm

        if frame_counter % 30 == 0:
            fps = 30 / (now_time - last_time)
            last_time = now_time

        if frame_counter % 5 == 0:
            display = frame.copy()
            cv2.rectangle(display, (10, 10), (300, 170), (0, 0, 0), -1)
            cv2.putText(display, "Freq: %.1f Hz" % current_freq, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(display, "Angle: %.2f deg" % tosa_angle, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(display, "MAX: %.2f deg" % max_angle_in_step, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 150, 255), 2)
            cv2.putText(display, "Thresh: %d" % current_threshold, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
            cv2.putText(display, "FPS: %.1f" % fps, (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("TOSA Live", display)
            cv2.imshow("Threshold", thresh)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(">>> CANCELLED BY USER.")
            break

finally:
    try:
        inst.write("OUTP OFF")
    except:
        pass
    csv_file.close()
    picam2.stop()
    cv2.destroyAllWindows()
    print(">>> System safely shut down.")
