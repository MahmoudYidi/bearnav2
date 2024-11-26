import tkinter as tk
from tkinter import messagebox
import os

def start_mapping():
    map_name = map_name_entry.get()
    if not map_name:
        messagebox.showerror("Input Error", "Please enter a map name.")
        return
    action_command = f"ros2 action send_goal /bearnav2/mapmaker bearnav2/action/MapMaker '{{map_name: \"{map_name}\", start: true}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Started mapping with map name '{map_name}'.")

def stop_mapping():
    map_name = map_name_entry.get()
    if not map_name:
        messagebox.showerror("Input Error", "Please enter a map name.")
        return
    action_command = f"ros2 action send_goal /bearnav2/mapmaker bearnav2/action/MapMaker '{{map_name: \"{map_name}\", start: false}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Stopped mapping with map name '{map_name}'.")

def repeat_mapping():
    map_name = map_name_entry.get()
    if not map_name:
        messagebox.showerror("Input Error", "Please enter a map name.")
        return
    action_command = f"ros2 action send_goal /bearnav2/repeater bearnav2/action/MapRepeater '{{map_name: \"{map_name}\"}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Repeated mapping with map name '{map_name}'.")

# Initialize the main window
root = tk.Tk()
root.title("ROS 2 Map Command GUI")
root.geometry("400x400")

# Map name label and entry field
map_name_label = tk.Label(root, text="Map Name:")
map_name_label.pack(pady=5)

map_name_entry = tk.Entry(root, width=30)
map_name_entry.pack(pady=5)

# Start/Stop Section
start_stop_frame = tk.LabelFrame(root, text="Start/Stop Mapping", padx=10, pady=10)
start_stop_frame.pack(padx=10, pady=10, fill="both")

# Start Mapping Button (Green)
start_button = tk.Button(start_stop_frame, text="Start Mapping", command=start_mapping, bg="green", fg="white", width=15)
start_button.pack(pady=5)

# Stop Mapping Button (Red)
stop_button = tk.Button(start_stop_frame, text="Stop Mapping", command=stop_mapping, bg="red", fg="white", width=15)
stop_button.pack(pady=5)

# Repeat Section
repeat_frame = tk.LabelFrame(root, text="Repeat Mapping", padx=10, pady=10)
repeat_frame.pack(padx=10, pady=10, fill="both")

# Repeat Mapping Button (Green)
repeat_button = tk.Button(repeat_frame, text="Repeat Mapping", command=repeat_mapping, bg="green", fg="white", width=15)
repeat_button.pack(pady=5)

# Start the GUI loop
root.mainloop()
