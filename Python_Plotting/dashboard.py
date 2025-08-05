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
ax2.set_ylim(-100, 100)
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
    send_pid_values()

def send_i_gain(val):
    pid_values['I'] = float(val)
    send_pid_values()

def send_d_gain(val):
    pid_values['D'] = float(val)
    send_pid_values()

root = tk.Tk()
root.title("PID Control")

p_scale = tk.Scale(root, from_=0.0, to=10.0, resolution=0.1, orient=tk.HORIZONTAL, label="P Gain", command=send_p_gain)
p_scale.pack()

i_scale = tk.Scale(root, from_=0.0, to=10.0, resolution=0.1, orient=tk.HORIZONTAL, label="I Gain", command=send_i_gain)
i_scale.pack()

d_scale = tk.Scale(root, from_=0.0, to=10.0, resolution=0.1, orient=tk.HORIZONTAL, label="D Gain", command=send_d_gain)
d_scale.pack()

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack()

ani = animation.FuncAnimation(fig, update, blit=True, interval=50)

root.mainloop()
