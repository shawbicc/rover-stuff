#!/usr/bin/env python3

import tkinter as tk
import ttkbootstrap as ttk
# from Tkinter import ttk
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

### Window
window = ttk.Window(themename='darkly')
window.title('Interplanetar GUI')
window.geometry('800x400')

window.columnconfigure(0, weight=1)
window.columnconfigure(1, weight=3)
window.columnconfigure(2, weight=2)

window.rowconfigure(0, weight=1)
window.rowconfigure(1, weight=2)
window.rowconfigure(2, weight=2)
window.rowconfigure(3, weight=2)
window.rowconfigure(4, weight=2)
window.rowconfigure(5, weight=2)
window.rowconfigure(6, weight=2)

## GLOBAL VARIABLES
global_font = tk.StringVar(value = 'Roboto Mono')
background_color = tk.StringVar(value = 'orange')
arming_status = tk.StringVar(value = 'disarmed')

# stats
arm_stat = tk.StringVar(value = 'Arming Status: --')
ping = tk.StringVar(value = 'Ping: --')
wheel_battery = tk.StringVar(value = 'Wheel Battery: --')
drive_mode = tk.StringVar(value='Driving Mode: --')

# wheel
speed_text = tk.StringVar(value = 'Speed: --')
speed = tk.DoubleVar(value = 0.7)
odom = tk.StringVar(value = 'Odometry: --')

# actions
arm_button_text = tk.StringVar(value = 'Arm')
need_delay = tk.StringVar(value = '1')

# arm
arm_pico_temp = tk.StringVar(value='Pico Temperature: --')
base_angle = tk.StringVar()
shoulder_angle = tk.StringVar()
elbow_angle = tk.StringVar()
pitch_angle = tk.StringVar()
roll_angle = tk.StringVar()
gripper_angle = tk.StringVar()


### FUCNTIONS

def update_color(color):
    header_label["background"] = color
    devicelist_label["background"] = color
    stats_label["background"] = color
    terminal_label["background"] = color
    arm_label["background"] = color
    actions_label["background"] = color
    


def arm_switch():
    # arming
    if arming_status.get() == 'disarmed':
        # add 5s delay if needed ------------------------> Has the delay but not doing what it's supposed to in the meantime
        if need_delay.get() == '1':
            arming_status.set('arming')   
            arm_stat.set('Arming Status: Arming')
            update_color('yellow')
            time.sleep(5)

		# start with stopped motion
        stop_moving()
		
        # change arming status
        arming_status.set('armed')
        arm_stat.set('Arming Status: Armed')
        arm_button_text.set('Disarm')
        
        # enable the widgets
        forward["state"] = "normal"
        backward["state"] = "normal"
        left["state"] = "normal"
        right["state"] = "normal"
        stop["state"] = "normal"
        speed_slider["state"] = "normal"
        
        # change color
        update_color('green')

    # disarming
    elif arming_status.get() == 'armed' or arming_status.get() == 'arming':
    	# stop moving
        stop_moving()
        # change arming status
        arming_status.set('disarmed')
        arm_stat.set('Arming Status: Disarmed')
        arm_button_text.set('Arm')
        
        # disable the widgets
        forward["state"] = "disabled"
        backward["state"] = "disabled"
        left["state"] = "disabled"
        right["state"] = "disabled"
        stop["state"] = "disabled"
        speed_slider["state"] = "disabled"
        
        # change color
        update_color('red')

def move_forward():
    data = Twist()
    data.linear.x = speed.get()
    data.angular.z = 0.0
    wheel_pub.publish(data)

def move_backward():
    data = Twist()
    data.linear.x = -1.0 * speed.get()
    data.angular.z = 0.0
    wheel_pub.publish(data)

def move_left():
    data = Twist()
    data.linear.x = 0.0
    data.angular.z = -1.0 * speed.get()
    wheel_pub.publish(data)

def move_right():
    data = Twist()
    data.linear.x = 0.0
    data.angular.z = speed.get()
    wheel_pub.publish(data)
    
def stop_moving():
    data = Twist()
    data.linear.x = 0.0
    data.angular.z = 0.0
    wheel_pub.publish(data)

def update_speed(value):
    speed.set(round(float(value), 2))
    speed_text.set("Speed: {}".format(round(float(value), 2)))

def show_joint_angles(data: Float64MultiArray):
    # base
    if data.data[0]==800:
        base_angle.set("Magnet not connected")

    elif data.data[0]==801:
        base_angle.set("Magnet Low")

    elif data.data[0]==802:
        base_angle.set("Magnet High")

    elif data.data[0]==900:
        base_angle.set("Encoder Not Connected")

    else:
        base_angle.set(str(data.data[0]))

    # shoulder
    if data.data[1]==800:
        shoulder_angle.set("Magnet not connected")

    elif data.data[1]==801:
        shoulder_angle.set("Magnet Low")

    elif data.data[1]==802:
        shoulder_angle.set("Magnet High")
    
    elif data.data[1]==900:
        shoulder_angle.set("Encoder Not Connected")

    else:
        shoulder_angle.set(str(data.data[1]))

    # elbow
    if data.data[2]==800:
        elbow_angle.set("Magnet not connected")

    elif data.data[2]==801:
        elbow_angle.set("Magnet Low")

    elif data.data[2]==802:
        elbow_angle.set("Magnet High")

    elif data.data[2]==900:
        elbow_angle.set("Encoder Not Connected")

    else:
        elbow_angle.set(str(data.data[2]))

    # pitch
    if data.data[4]==800:
        pitch_angle.set("Magnet not connected")

    elif data.data[4]==801:
        pitch_angle.set("Magnet Low")

    elif data.data[4]==802:
        pitch_angle.set("Magnet High")

    elif data.data[4]==900:
        pitch_angle.set("Encoder Not Connected")

    else:
        pitch_angle.set(str(data.data[4]))

    # roll
    if data.data[5]==800:
        roll_angle.set("Magnet not connected")

    elif data.data[5]==801:
        roll_angle.set("Magnet Low")

    elif data.data[5]==802:
        roll_angle.set("Magnet High")

    elif data.data[5]==900:
        roll_angle.set("Encoder Not Connected")

    else:
        roll_angle.set(str(data.data[5]))

    # temp
    if data.data[6]==-1:
        arm_pico_temp.set('Emergency stopped \nfrom pico BOOTSEL Button')
    else:
        arm_pico_temp.set("Pico Temperature: {}".format(data.data[6]))


header_frame = ttk.Frame(window)
devicelist_frame = ttk.Frame(window)
stats_frame = ttk.Frame(window)
controls_frame = ttk.Frame(window)
terminal_frame = ttk.Frame(window)
arm_frame = ttk.Frame(window)
actions_frame = ttk.Frame(window)


header_label = ttk.Label(header_frame, anchor=tk.CENTER, borderwidth=2, relief="solid", font=(global_font.get(), 16, "bold"), text='Team Interplanetar', background=background_color.get())

devicelist_label = ttk.Label(devicelist_frame, text='Devices', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

stats_label =  ttk.Label(stats_frame, text='Stats', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

# stat labels (might have update issues)
arm_status =  ttk.Label(stats_frame, textvariable=arm_stat, font=(global_font.get(), 10))
drive_mode_status = ttk.Label(stats_frame, textvariable=drive_mode, font=(global_font.get(), 10))
ping_count = ttk.Label(stats_frame, textvariable=ping, font=(global_font.get(), 10))
wheel_bat = ttk.Label(stats_frame, textvariable=wheel_battery, font=(global_font.get(), 10))
sensors = ttk.Label(stats_frame, text='Sensors:', font=(global_font.get(), 10))
pico_temp = ttk.Label(stats_frame, textvariable=arm_pico_temp, font=(global_font.get(), 10))

# packing
stats_label.pack(expand=False, fill='x')
arm_status.pack(expand=False, fill='x')
drive_mode_status.pack(expand=False, fill='x')
ping_count.pack(expand=False, fill='x')
wheel_bat.pack(expand=False, fill='x')
sensors.pack(expand=False, fill='x')
pico_temp.pack(expand=False, fill='x')

controls_notebook = ttk.Notebook(controls_frame)

### Wheel Tab
wheel_tab = ttk.Frame(controls_notebook)

wheel_tab.rowconfigure(0, weight=1)
wheel_tab.rowconfigure(1, weight=1)
wheel_tab.rowconfigure(2, weight=1)
wheel_tab.rowconfigure(3, weight=1)
wheel_tab.rowconfigure(4, weight=1)
wheel_tab.rowconfigure(5, weight=1)

wheel_tab.columnconfigure(0, weight=1)
wheel_tab.columnconfigure(1, weight=1)
wheel_tab.columnconfigure(2, weight=1)
wheel_tab.columnconfigure(3, weight=1)
wheel_tab.columnconfigure(4, weight=1)
wheel_tab.columnconfigure(5, weight=1)
wheel_tab.columnconfigure(6, weight=1)
wheel_tab.columnconfigure(7, weight=1)

forward = ttk.Button(wheel_tab, text="Forward", bootstyle='success', state='disabled', command=move_forward)
backward = ttk.Button(wheel_tab, text="Backward", bootstyle='success', state='disabled', command=move_backward)
left = ttk.Button(wheel_tab, text="Left", bootstyle='success', state='disabled', command=move_left)
right = ttk.Button(wheel_tab, text="Right", bootstyle='success', state='disabled', command=move_right)
stop = ttk.Button(wheel_tab, text="Stop", bootstyle='success', state='disabled', command=stop_moving)

speed_val = ttk.Label(wheel_tab, textvariable=speed_text, font=(global_font.get(), 10))
speed_slider = ttk.Scale(wheel_tab, command = update_speed, state='disabled')
odom_text = ttk.Label(wheel_tab, textvariable=odom, font=(global_font.get(), 10))

forward.grid(row=1, column=1, sticky='news')
backward.grid(row=3, column=1, sticky='news')
left.grid(row=2, column=0, sticky='news')
right.grid(row=2, column=2, sticky='news')
stop.grid(row=2, column=1, sticky='news')
speed_val.grid(row=0, column=4, columnspan=3, sticky='news')
speed_slider.grid(row=1, column=4, columnspan=3, sticky='news')
odom_text.grid(row=2, column=4, columnspan=3, rowspan=2, sticky='news')


### Arm Tab
arm_tab = ttk.Frame(controls_notebook)

# arm_tab.rowconfigure(0, weight=1)
# arm_tab.rowconfigure(1, weight=1)
# arm_tab.rowconfigure(2, weight=1)
# arm_tab.rowconfigure(3, weight=1)
# arm_tab.rowconfigure(4, weight=1)
# arm_tab.rowconfigure(5, weight=1)
# arm_tab.rowconfigure(6, weight=1)
# arm_tab.rowconfigure(7, weight=1)

# arm_tab.columnconfigure(0, weight=1)
# arm_tab.columnconfigure(1, weight=1)
# arm_tab.columnconfigure(2, weight=1)
# arm_tab.columnconfigure(3, weight=1)
# arm_tab.columnconfigure(4, weight=1)
# arm_tab.columnconfigure(5, weight=1)
# arm_tab.columnconfigure(6, weight=1)
# arm_tab.columnconfigure(7, weight=1)
# arm_tab.columnconfigure(8, weight=1)
# arm_tab.columnconfigure(9, weight=2)

# joint_label = ttk.Label(arm_tab, text='Joint Angles', font=(global_font.get(), 12, 'bold'))
# base_label = ttk.Label(arm_tab, text='Base: ', font=(global_font.get(), 10))
# shoulder_label = ttk.Label(arm_tab, text='Shoulder: ', font=(global_font.get(), 10))
# elbow_label = ttk.Label(arm_tab, text='Elbow: ', font=(global_font.get(), 10))
# pitch_label = ttk.Label(arm_tab, text='Pitch: ', font=(global_font.get(), 10))
# roll_label = ttk.Label(arm_tab, text='Roll: ', font=(global_font.get(), 10))
# gripper_label = ttk.Label(arm_tab, text='Gripper: ', font=(global_font.get(), 10))

# joint_label.grid(row=0, column=0, columnspan=8, sticky='news')
# base_label.grid(row=1, column=0, sticky='news')
# shoulder_label.grid(row=2, column=0, sticky='news')
# elbow_label.grid(row=3, column=0, sticky='news')
# pitch_label.grid(row=4, column=0, sticky='news')
# roll_label.grid(row=5, column=0, sticky='news')
# gripper_label.grid(row=6, column=0, sticky='news')



### Science Tab
science_tab = ttk.Frame(controls_notebook)

controls_notebook.add(wheel_tab, text="Wheel")
controls_notebook.add(arm_tab, text="Arm")
controls_notebook.add(science_tab, text="Science tools")

controls_notebook.pack(expand=True, fill='both')

terminal_label = ttk.Label(terminal_frame, text='Terminal', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

### Arm Frame
arm_frame.rowconfigure(0, weight=1)
arm_frame.rowconfigure(1, weight=1)
arm_frame.rowconfigure(2, weight=1)
arm_frame.rowconfigure(3, weight=1)
arm_frame.rowconfigure(4, weight=1)
arm_frame.rowconfigure(5, weight=1)
arm_frame.rowconfigure(6, weight=1)
arm_frame.rowconfigure(7, weight=1)

arm_frame.columnconfigure(0, weight=1)
arm_frame.columnconfigure(1, weight=3)
arm_frame.columnconfigure(2, weight=1)

arm_label = ttk.Label(arm_frame, text='Rover Arm', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))
base_label = ttk.Label(arm_frame, text='Base: ', font=(global_font.get(), 10))
shoulder_label = ttk.Label(arm_frame, text='Shoulder: ', font=(global_font.get(), 10))
elbow_label = ttk.Label(arm_frame, text='Elbow: ', font=(global_font.get(), 10))
pitch_label = ttk.Label(arm_frame, text='Pitch: ', font=(global_font.get(), 10))
roll_label = ttk.Label(arm_frame, text='Roll: ', font=(global_font.get(), 10))
gripper_label = ttk.Label(arm_frame, text='Gripper: ', font=(global_font.get(), 10))

base_value = ttk.Label(arm_frame, textvariable=base_angle, font=(global_font.get(), 10))
shoulder_value = ttk.Label(arm_frame, textvariable=shoulder_angle, font=(global_font.get(), 10))
elbow_value = ttk.Label(arm_frame, textvariable=elbow_angle, font=(global_font.get(), 10))
pitch_value = ttk.Label(arm_frame, textvariable=pitch_angle, font=(global_font.get(), 10))
roll_value = ttk.Label(arm_frame, textvariable=roll_angle, font=(global_font.get(), 10))
gripper_value = ttk.Label(arm_frame, textvariable=gripper_angle, font=(global_font.get(), 10))

arm_label.grid(row=0, column=0, columnspan=3, sticky='news')
base_label.grid(row=1, column=0, sticky='news')
shoulder_label.grid(row=2, column=0, sticky='news')
elbow_label.grid(row=3, column=0, sticky='news')
pitch_label.grid(row=4, column=0, sticky='news')
roll_label.grid(row=5, column=0, sticky='news')
gripper_label.grid(row=6, column=0, sticky='news')

base_value.grid(row=1, column=1, sticky='news')
shoulder_value.grid(row=2, column=1, sticky='news')
elbow_value.grid(row=3, column=1, sticky='news')
pitch_value.grid(row=4, column=1, sticky='news')
roll_value.grid(row=5, column=1, sticky='news')
gripper_value.grid(row=6, column=1, sticky='news')






actions_label = ttk.Label(actions_frame, text='Actions', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

arm_button = tk.Button(actions_frame, textvariable=arm_button_text, command=arm_switch, width=20)
timer_checkbox = tk.Checkbutton(actions_frame, text='Add 5s delay before arming', variable=need_delay)

actions_label.pack(expand=False, fill='x')
arm_button.pack(padx=5, pady=10)
timer_checkbox.pack()

header_label.pack(expand=True, fill='both')
devicelist_label.pack(expand=False, fill='x')

# controls_label.pack(expand=True, fill='both')
terminal_label.pack(expand=False, fill='x')

### Frames Packing
header_frame.grid(row=0, column=0, columnspan=3, sticky='news')
stats_frame.grid(row=1, column=0, rowspan=4, sticky='news')
devicelist_frame.grid(row=5, column=0, rowspan=2, sticky='news')
controls_frame.grid(row=1, column=1, rowspan=4, sticky='news')
terminal_frame.grid(row=5, column=1, rowspan=2, sticky='news')
arm_frame.grid(row=1, column=2, rowspan=4, sticky='news')
actions_frame.grid(row=5, column=2, rowspan=2, sticky='news')

### ROS
rospy.init_node("nav_pub", anonymous=True)
wheel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
arm_sub = rospy.Subscriber('arm_status_notify', Float64MultiArray, show_joint_angles)

window.mainloop()


