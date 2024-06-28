import tkinter as tk
# from PIL import Image, ImageTk


def increment_slider(event):
    current_value = int(slider.get())
    new_value = min(current_value + 1, 100)  # Ensure the new value does not exceed the maximum value
    slider.set(new_value)
    send_to_arduino(new_value + 9)  # Send updated value to Arduino

def decrement_slider(event):
    current_value = int(slider.get())
    new_value = max(current_value - 1, 0)  # Ensure the new value does not go below the minimum value
    slider.set(new_value)
    send_to_arduino(new_value + 9)  # Send updated value to Arduino


def toggle_button(button,button2,button4,button5,button6,button7,button8):
    if button3["text"] == "Start":
        send_to_arduino(3)
       
        button3["text"] = "Stop"
        button3["bg"] = "red"
    else:
        send_to_arduino(4)
        button["bg"] = "yellow"
        button2["bg"] = "yellow"
        button4["bg"] = "yellow"
        button5["bg"] = "yellow"
        button6["bg"] = "yellow"
        button7["bg"] = "yellow"
        button8["bg"] = "yellow"
        
        button3["text"] = "Start"
        button3["bg"] = "green"

def button_click(button,button2,value):
    button.config(bg="green")
    
    button2.config(bg="yellow")
    
    send_to_arduino(value)
      # Send 1 to Arduino when button is clicked
def on_space(event):
    toggle_button()

def button_click2(button,button2,button3,button4,button5,value):
    button.config(bg="green")
    button2.config(bg="yellow")
    button3.config(bg="yellow")
    button4.config(bg="yellow")
    button5.config(bg="yellow")
    
    send_to_arduino(value)
      # Send 1 to Arduino when button is clicked



def send_to_arduino(value):
    print(value)
    # pub.publish(value)
    

def update_gui(angle):
    angle_label.config(text="motor 1 angle:" + str(angle))
# def callback(data):
#     update_gui(data)
# rospy.Subscriber("encoder_angle", Int32, callback)


# Create the GUI
root = tk.Tk()
root.title("Send Numbers to Arduino")
window_width = 300
window_height = 300
root.geometry(f"{window_width}x{window_height}")
button_size = 40
button_color = "green"

# button = tk.Button(root, text="Click", command=send_to_arduino(1), width=button_size, height=button_size, bg=button_color, bd=0, highlightthickness=0, activebackground=button_color)
# button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
canvas = tk.Canvas(root, width=window_width, height=window_height, bg="#add8e6", highlightthickness=0)
canvas.pack()

button = tk.Button(root, text="cw", bg="yellow")
button.place(x=100, y=50)  # Place the button at coordinates (100, 50)
button2 = tk.Button(root, text="acw", bg="yellow")
button2.place(x=150, y=50)  # Place the button at coordinates (100, 50)
button3 = tk.Button(root, text="Start", bg="green")
button3.place(x=200, y=50)  # Place the button at coordinates (100, 50)

button4 = tk.Button(root, text="DC1", bg="yellow")
button4.place(x=50, y=20)  # Place the button at coordinates (100, 50)
button5 = tk.Button(root, text="DC2", bg="yellow")
button5.place(x=90, y=20)  # Place the button at coordinates (100, 50)
button6 = tk.Button(root, text="Servo1", bg="yellow")
button6.place(x=140, y=20)  # Place the button at coordinates (100, 50)
button7 = tk.Button(root, text="servo2", bg="yellow")
button7.place(x=190, y=20)  # Place the button at coordinates (100, 50)
button8 = tk.Button(root, text="Drill", bg="yellow")
button8.place(x=240, y=20)  # Place the button at coordinates (100, 50)


button.config(command=lambda:button_click(button,button2,1))
button2.config(command=lambda:button_click(button2,button,2))
button3.config(command=lambda:toggle_button(button4,button5,button6,button7,button,button2,button8))
root.bind("<space>", on_space) 

button4.config(command=lambda:button_click2(button4,button5,button6,button7,button8,5))
button5.config(command=lambda:button_click2(button5,button4,button6,button7,button8,6))
button6.config(command=lambda:button_click2(button6,button5,button4,button7,button8,7))
button7.config(command=lambda:button_click2(button7,button5,button6,button4,button8,8))
button8.config(command=lambda:button_click2(button8,button5,button6,button4,button7,9))



slider = tk.Scale(root, from_=0, to=100, orient="horizontal", command = lambda value: send_to_arduino((int(value) + 10)))

slider.place(x=100, y=110)

root.bind("<Right>", increment_slider)  # Bind right arrow key to increment_slider function
root.bind("<Left>", decrement_slider)   # Bind left arrow key to decrement_slider function

angle_label = tk.Label(root, text="motor 1 angle: ")
angle_label.place(x=10, y=180)
angle_label2 = tk.Label(root, text="motor 2 angle: ")
angle_label2.place(x=10, y=210)
angle_label3 = tk.Label(root, text="servo 1: ")
angle_label3.place(x=10, y=240)
angle_label4 = tk.Label(root, text="servo 2: ")
angle_label4.place(x=10, y=270)

angle_label4 = tk.Label(root, text="weight ")
angle_label4.place(x=200, y=270)


root.mainloop()