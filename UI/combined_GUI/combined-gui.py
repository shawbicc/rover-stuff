
import tkinter as tk
import ttkbootstrap as ttk
import time

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


global_font = tk.StringVar(value = 'Roboto Mono')
background_color = tk.StringVar(value = 'orange')
arming_status = tk.StringVar(value = 'disarmed')

# stats
arm_stat = tk.StringVar(value = 'Arming Status: --')
ping = tk.StringVar(value = 'Ping: --')
wheel_battery = tk.StringVar(value = 'Wheel Battery: --')

# wheel
speed_text = tk.StringVar(value = 'Speed: --')
speed = tk.DoubleVar(value = 0.4)
odom = tk.StringVar(value = 'Odometry: --')

# actions
arm_button_text = tk.StringVar(value = 'Arm')
need_delay = tk.StringVar(value = '1')

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
        # add 5s delay if needed
        if need_delay.get() == '1':
            arming_status.set('arming')   
            arm_stat.set('Arming Status: Arming')
            update_color('yellow')
            for i in range(5):
                arm_button_text.set(f'Arming in {i}...')
                time.sleep(1)

        # change arming status
        arming_status.set('armed')
        arm_stat.set('Arming Status: Armed')
        arm_button_text.set('Disarm')
        
        # enable the widgets
        forward["state"] = "normal"
        backward["state"] = "normal"
        left["state"] = "normal"
        right["state"] = "normal"
        speed_slider["state"] = "normal"
        
        # change color
        update_color('green')

    # disarming
    elif arming_status.get() == 'armed' or arming_status.get() == 'arming':
        # change arming status
        arming_status.set('disarmed')
        arm_stat.set('Arming Status: Disarmed')
        arm_button_text.set('Arm')
        
        # disable the widgets
        forward["state"] = "disabled"
        backward["state"] = "disabled"
        left["state"] = "disabled"
        right["state"] = "disabled"
        speed_slider["state"] = "disabled"
        
        # change color
        update_color('red')

def move_forward():
    print('moving forward')

def move_backward():
    print('moving backward')

def move_left():
    print('moving left')

def move_right():
    print('moving right')

def update_speed(value):
    speed.set(round(float(value), 2))
    speed_text.set(f'Speed: {round(float(value), 2)}')


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
ping_count = ttk.Label(stats_frame, textvariable=ping, font=(global_font.get(), 10))
wheel_bat = ttk.Label(stats_frame, textvariable=wheel_battery, font=(global_font.get(), 10))
sensors = ttk.Label(stats_frame, text='Sensors:', font=(global_font.get(), 10))


# packing
stats_label.pack(expand=False, fill='x')
arm_status.pack(expand=False, fill='x')
ping_count.pack(expand=False, fill='x')
wheel_bat.pack(expand=False, fill='x')
sensors.pack(expand=False, fill='x')

controls_notebook = ttk.Notebook(controls_frame)

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

speed_val = ttk.Label(wheel_tab, textvariable=speed_text, font=(global_font.get(), 10))
# speed_slider = ttk.Scale(wheel_tab, command = lambda value: speed.set(f'Speed: {round(float(value), 2)}'), state='disabled')
speed_slider = ttk.Scale(wheel_tab, command = update_speed, state='disabled')
odom_text = ttk.Label(wheel_tab, textvariable=odom, font=(global_font.get(), 10))

forward.grid(row=1, column=1, sticky='news')
backward.grid(row=3, column=1, sticky='news')
left.grid(row=2, column=0, sticky='news')
right.grid(row=2, column=2, sticky='news')
speed_val.grid(row=0, column=4, columnspan=3, sticky='news')
speed_slider.grid(row=1, column=4, columnspan=3, sticky='news')
odom_text.grid(row=2, column=4, columnspan=3, rowspan=2, sticky='news')

arm_tab = ttk.Frame(controls_notebook)

science_tab = ttk.Frame(controls_notebook)

controls_notebook.add(wheel_tab, text="Wheel")
controls_notebook.add(arm_tab, text="Arm")
controls_notebook.add(science_tab, text="Science tools")

controls_notebook.pack(expand=True, fill='both')

terminal_label = ttk.Label(terminal_frame, text='Terminal', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

arm_label = ttk.Label(arm_frame, text='Rover Arm', borderwidth=2, relief="solid", background=background_color.get(), anchor=tk.CENTER, font=(global_font.get(), 12, 'bold'))

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
arm_label.pack(expand=False, fill='x')


header_frame.grid(row=0, column=0, columnspan=3, sticky='news')
stats_frame.grid(row=1, column=0, rowspan=4, sticky='news')
devicelist_frame.grid(row=5, column=0, rowspan=2, sticky='news')
controls_frame.grid(row=1, column=1, rowspan=4, sticky='news')
terminal_frame.grid(row=5, column=1, rowspan=2, sticky='news')
arm_frame.grid(row=1, column=2, rowspan=4, sticky='news')
actions_frame.grid(row=5, column=2, rowspan=2, sticky='news')

window.mainloop()


