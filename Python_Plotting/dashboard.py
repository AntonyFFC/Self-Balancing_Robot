import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

UDP_IP = "0.0.0.0"
UDP_PORT = 7777
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

ESP_IP = "192.168.1.22"
ESP_PORT = 7778
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

x_data = []
pitch_data = []
setpitch_data = []
u_data = []

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

# Pitch comparison plot
line1, = ax1.plot([], [], 'b-', label='Pitch', linewidth=2)
line2, = ax1.plot([], [], 'r-', label='Set Pitch', linewidth=2)
ax1.set_ylim(-90, 90)
ax1.set_xlim(0, 100)
ax1.set_ylabel("Pitch (degrees)")
ax1.legend()
ax1.grid(True)
ax1.set_title("Pitch vs Set Pitch")

# Control Signal plot
line3, = ax2.plot([], [], 'g-', label='Control Signal (u)', linewidth=2)
ax2.set_ylim(-250, 250)
ax2.set_xlim(0, 100)
ax2.set_ylabel("Control Signal")
ax2.set_xlabel("Time")
ax2.legend()
ax2.grid(True)
ax2.set_title("Control Signal")

def update(frame):
    data, addr = sock.recvfrom(1024)
    values = data.decode().strip().split(",")
    
    if len(values) >= 3:
        pitch = float(values[0])
        setPitch = float(values[1])
        u = float(values[2])
        
        x_data.append(len(x_data))
        pitch_data.append(pitch)
        setpitch_data.append(setPitch)
        u_data.append(u)
        
        line1.set_data(x_data, pitch_data)
        line2.set_data(x_data, setpitch_data)
        line3.set_data(x_data, u_data)
        
        xlim_max = max(100, len(x_data))
        ax1.set_xlim(0, xlim_max)
        ax2.set_xlim(0, xlim_max)
        
        return line1, line2, line3
    
    return line1, line2, line3

pid_values = {'P': 0.0, 'I': 0.0, 'D': 0.0}

def send_pid_values():
    cmd = f"P={pid_values['P']},I={pid_values['I']},D={pid_values['D']}\n"
    print(f"Sending command '{cmd.strip()}' to {ESP_IP}:{ESP_PORT}")
    try:
        send_sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))
        print("Command sent successfully")
    except Exception as e:
        print(f"Error sending command: {e}")

def send_p_gain(val):
    pid_values['P'] = float(val)
    p_entry.delete(0, tk.END)
    p_entry.insert(0, f"{float(val):.1f}")
    send_pid_values()

def send_i_gain(val):
    pid_values['I'] = float(val)
    i_entry.delete(0, tk.END)
    i_entry.insert(0, f"{float(val):.1f}")
    send_pid_values()

def send_d_gain(val):
    pid_values['D'] = float(val)
    d_entry.delete(0, tk.END)
    d_entry.insert(0, f"{float(val):.1f}")
    send_pid_values()

def on_p_entry_change(event):
    try:
        val = float(p_entry.get())
        if 0.0 <= val <= 10.0:
            p_scale.set(val)
            pid_values['P'] = val
            send_pid_values()
    except ValueError:
        pass

def on_i_entry_change(event):
    try:
        val = float(i_entry.get())
        if 0.0 <= val <= 10.0:
            i_scale.set(val)
            pid_values['I'] = val
            send_pid_values()
    except ValueError:
        pass

def on_d_entry_change(event):
    try:
        val = float(d_entry.get())
        if 0.0 <= val <= 10.0:
            d_scale.set(val)
            pid_values['D'] = val
            send_pid_values()
    except ValueError:
        pass

root = tk.Tk()
root.title("PID Control")

pid_frame = tk.Frame(root)
pid_frame.pack(pady=10)

# P Gain controls
p_frame = tk.Frame(pid_frame)
p_frame.pack(side=tk.LEFT, padx=10)
tk.Label(p_frame, text="P Gain").pack()
p_scale = tk.Scale(p_frame, from_=0.0, to=10.0, resolution=0.1, orient=tk.VERTICAL, command=send_p_gain)
p_scale.pack()
p_entry = tk.Entry(p_frame, width=8)
p_entry.pack(pady=5)

# I Gain controls
i_frame = tk.Frame(pid_frame)
i_frame.pack(side=tk.LEFT, padx=10)
tk.Label(i_frame, text="I Gain").pack()
i_scale = tk.Scale(i_frame, from_=0.0, to=10.0, resolution=0.1, orient=tk.VERTICAL, command=send_i_gain)
i_scale.pack()
i_entry = tk.Entry(i_frame, width=8)
i_entry.pack(pady=5)

# D Gain controls
d_frame = tk.Frame(pid_frame)
d_frame.pack(side=tk.LEFT, padx=10)
tk.Label(d_frame, text="D Gain").pack()
d_scale = tk.Scale(d_frame, from_=0.0, to=10.0, resolution=0.1, orient=tk.VERTICAL, command=send_d_gain)
d_scale.pack()
d_entry = tk.Entry(d_frame, width=8)
d_entry.pack(pady=5)

# Bind entry fields to update functions
p_entry.bind('<Return>', on_p_entry_change)
p_entry.bind('<FocusOut>', on_p_entry_change)
i_entry.bind('<Return>', on_i_entry_change)
i_entry.bind('<FocusOut>', on_i_entry_change)
d_entry.bind('<Return>', on_d_entry_change)
d_entry.bind('<FocusOut>', on_d_entry_change)

p_entry.insert(0, "0.0")
i_entry.insert(0, "0.0")
d_entry.insert(0, "0.0")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack()

ani = animation.FuncAnimation(fig, update, blit=True, interval=50)

root.mainloop()
