import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import csv
import datetime

max_length = 500

UDP_IP = "0.0.0.0"
UDP_PORT = 7777
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.01)

ESP_IP = "192.168.1.14"
ESP_PORT = 7778
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

x_data = []
pitch_data = []
setpitch_data = []
u_data = []

csv_data = []
start_time = datetime.datetime.now()
csv_filename = f"pid_data_{start_time.strftime('%Y%m%d_%H%M%S')}.csv"

error_counts = {
    'READ_TIMEOUT': 0,
    'WRITE_TIMEOUT': 0,
    'READ_ERROR': 0,
    'WRITE_ERROR': 0,
    'SENSOR_READ_FAILED': 0
}
total_errors = 0
last_error_time = ""
last_error_code = ""

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Pitch comparison plot
line1, = ax1.plot([], [], 'b-', label='Pitch', linewidth=2)
line2, = ax1.plot([], [], 'r-', label='Set Pitch', linewidth=2)
ax1.set_ylim(-60, 60)
ax1.set_xlim(0, 500)
ax1.set_ylabel("Pitch (degrees)")
ax1.legend()
ax1.grid(True)
ax1.set_title("Pitch vs Set Pitch")

pitch_text = ax1.text(0.02, 0.95, 'Pitch: -- °', transform=ax1.transAxes, 
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7),
                     fontsize=10, verticalalignment='top')
setpitch_text = ax1.text(0.02, 0.85, 'Set Pitch: -- °', transform=ax1.transAxes,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7),
                        fontsize=10, verticalalignment='top')

# Control Signal plot
line3, = ax2.plot([], [], 'g-', label='Control Signal (u)', linewidth=2)
ax2.set_ylim(-150, 150)
ax2.set_xlim(0, 500)
ax2.set_ylabel("Control Signal")
ax2.set_xlabel("Time")
ax2.legend()
ax2.grid(True)
ax2.set_title("Control Signal")


control_text = ax2.text(0.02, 0.95, 'Control Signal: --', transform=ax2.transAxes,
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7),
                       fontsize=10, verticalalignment='top')

def update(frame):
    global x_data, pitch_data, setpitch_data, u_data
    try:
        data, addr = sock.recvfrom(1024)
        message = data.decode().strip()
        
        if message.startswith("I2C_ERROR:"):
            parse_error_message(message)
            return line1, line2, line3, pitch_text, setpitch_text, control_text
        
        if message.startswith("INIT:"):
            parse_init_message(message)
            return line1, line2, line3, pitch_text, setpitch_text, control_text
        
        values = message.split(",")
        
        if len(values) >= 3:
            pitch = float(values[0])
            setPitch = float(values[1])
            u = float(values[2])
            
            elapsed_time = (datetime.datetime.now() - start_time).total_seconds()
            
            x_data.append(len(x_data))
            pitch_data.append(pitch)
            setpitch_data.append(setPitch)
            u_data.append(u)

            if len(x_data) > 500:
                x_data.pop(0)
                pitch_data.pop(0)
                setpitch_data.pop(0)
                u_data.pop(0)
                
                x_data = [i for i in range(len(x_data))]

            if save_to_csv.get():
                csv_data.append([elapsed_time, pitch, setPitch, u, pid_values['P'], pid_values['I'], pid_values['D'], pid_values['MP']])
            
            line1.set_data(x_data, pitch_data)
            line2.set_data(x_data, setpitch_data)
            line3.set_data(x_data, u_data)
            
            pitch_text.set_text(f'Pitch: {pitch:.1f}°')
            setpitch_text.set_text(f'Set Pitch: {setPitch:.1f}°')
            control_text.set_text(f'Control Signal: {u:.1f}')
            
            ax1.set_xlim(0, 500)
            ax2.set_xlim(0, 500)
            
    except socket.timeout:
        pass
    except Exception as e:
        print(f"Socket error: {e}")
    
    return line1, line2, line3, pitch_text, setpitch_text, control_text

def parse_error_message(message):
    global total_errors, last_error_time, last_error_code
    
    try:
        # np Parse: "I2C_ERROR:READ_TIMEOUT,code=0x107,total=5"
        parts = message.split(":")
        if len(parts) >= 2:
            error_info = parts[1]
            error_parts = error_info.split(",")
            
            if len(error_parts) >= 3:
                error_type = error_parts[0]
                code_part = error_parts[1] 
                total_part = error_parts[2]
                
                if "=" in code_part:
                    last_error_code = code_part.split("=")[1]
                
                if "=" in total_part:
                    esp_total = int(total_part.split("=")[1])
                    total_errors = esp_total
                
                if error_type in error_counts:
                    error_counts[error_type] += 1
                
                last_error_time = datetime.datetime.now().strftime("%H:%M:%S")
                
                update_error_display()
                
                print(f"I2C Error: {error_type}, Code: {last_error_code}, Total: {total_errors}")
                
    except Exception as e:
        print(f"Error parsing error message: {e}")

def parse_init_message(message):
    """Parse initial PID values from ESP32"""
    try:
        # np Parse: "INIT:P=2.500,I=0.000001,D=0.000,MP=0.0"
        parts = message.split(":")
        if len(parts) >= 2:
            pid_info = parts[1]
            
            p_pos = pid_info.find("P=")
            if p_pos != -1:
                p_end = pid_info.find(",", p_pos)
                if p_end == -1:
                    p_end = len(pid_info)
                p_val = float(pid_info[p_pos+2:p_end])
                pid_values['P'] = p_val
                p_scale.set(p_val)
                p_entry.delete(0, tk.END)
                p_entry.insert(0, f"{p_val:.1f}")
            
            i_pos = pid_info.find("I=")
            if i_pos != -1:
                i_end = pid_info.find(",", i_pos)
                if i_end == -1:
                    i_end = len(pid_info)
                i_val = float(pid_info[i_pos+2:i_end])
                pid_values['I'] = i_val
                i_scale.set(i_val)
                i_entry.delete(0, tk.END)
                i_entry.insert(0, f"{i_val:.6f}")
            
            d_pos = pid_info.find("D=")
            if d_pos != -1:
                d_end = pid_info.find(",", d_pos)
                if d_end == -1:
                    d_end = len(pid_info)
                d_val = float(pid_info[d_pos+2:d_end])
                pid_values['D'] = d_val
                d_scale.set(d_val)
                d_entry.delete(0, tk.END)
                d_entry.insert(0, f"{d_val:.1f}")
            
            mp_pos = pid_info.find("MP=")
            if mp_pos != -1:
                mp_end = pid_info.find(",", mp_pos)
                if mp_end == -1:
                    mp_end = len(pid_info)
                mp_val = float(pid_info[mp_pos+3:mp_end])
                pid_values['MP'] = mp_val
                mp_scale.set(mp_val)
                mp_entry.delete(0, tk.END)
                mp_entry.insert(0, f"{mp_val:.1f}")
            
            print(f"Received PID values - P:{pid_values['P']}, I:{pid_values['I']}, D:{pid_values['D']}, MP:{pid_values['MP']}")
                
    except Exception as e:
        print(f"Error parsing init message: {e}")

def update_error_display():
    try:
        total_label.config(text=f"Total Errors: {total_errors}")
        last_error_label.config(text=f"Last Error: {last_error_time}")
        error_code_label.config(text=f"Last Code: {last_error_code}")
        
        read_timeout_label.config(text=f"Read Timeouts: {error_counts['READ_TIMEOUT']}")
        write_timeout_label.config(text=f"Write Timeouts: {error_counts['WRITE_TIMEOUT']}")
        read_error_label.config(text=f"Read Errors: {error_counts['READ_ERROR']}")
        write_error_label.config(text=f"Write Errors: {error_counts['WRITE_ERROR']}")
        sensor_error_label.config(text=f"Sensor Failures: {error_counts['SENSOR_READ_FAILED']}")
        
        flash_error_display()
    except:
        pass

def flash_error_display():
    try:
        error_labels = [
            total_label, last_error_label, error_code_label,
            read_timeout_label, write_timeout_label, read_error_label,
            write_error_label, sensor_error_label
        ]
        
        for label in error_labels:
            label.config(bg="red", fg="white")

        root.after(1000, reset_error_display)
    except:
        pass

def reset_error_display():
    try:
        error_labels = [
            total_label, last_error_label, error_code_label,
            read_timeout_label, write_timeout_label, read_error_label,
            write_error_label, sensor_error_label
        ]
        
        for label in error_labels:
            label.config(bg=root.cget('bg'), fg="black")
    except:
        pass

pid_values = {'P': 0.0, 'I': 900000.0, 'D': 0.0, 'MP': 0.0}

def send_pid_values():
    cmd = f"P={pid_values['P']},I={pid_values['I']},D={pid_values['D']},MP={pid_values['MP']}\n"
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

def send_i_gain(val):
    pid_values['I'] = float(val)
    i_entry.delete(0, tk.END)
    i_entry.insert(0, f"{float(val):.1f}")

def send_d_gain(val):
    pid_values['D'] = float(val)
    d_entry.delete(0, tk.END)
    d_entry.insert(0, f"{float(val):.1f}")

def send_min_pwm(val):
    pid_values['MP'] = float(val)
    mp_entry.delete(0, tk.END)
    mp_entry.insert(0, f"{float(val):.1f}")

def send_pid_button_click():
    send_pid_values()

def get_pid_values():
    cmd = "GET\n"
    print(f"Requesting PID values from {ESP_IP}:{ESP_PORT}")
    try:
        send_sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))
        print("GET command sent successfully")
    except Exception as e:
        print(f"Error sending GET command: {e}")

def on_p_entry_change(event):
    try:
        val = float(p_entry.get())
        if 0.0 <= val <= 40.0:
            p_scale.set(val)
            pid_values['P'] = val
    except ValueError:
        pass

def on_i_entry_change(event):
    try:
        val = float(i_entry.get())
        if 0.0 <= val <= 900000.0:
            i_scale.set(val)
            pid_values['I'] = val
    except ValueError:
        pass

def on_d_entry_change(event):
    try:
        val = float(d_entry.get())
        if 0.0 <= val <= 10.0:
            d_scale.set(val)
            pid_values['D'] = val
    except ValueError:
        pass

def on_mp_entry_change(event):
    try:
        val = float(mp_entry.get())
        if 0.0 <= val <= 100.0:
            mp_scale.set(val)
            pid_values['MP'] = val
    except ValueError:
        pass

root = tk.Tk()
root.title("PID Control")

save_to_csv = tk.BooleanVar()
save_to_csv.set(True)

def on_closing():
    if save_to_csv.get() and len(csv_data) > 0:
        print("Saving data to CSV...")
        try:
            with open(csv_filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time', 'Pitch', 'SetPitch', 'ControlSignal', 'P_Gain', 'I_Gain', 'D_Gain', 'Min_PWM_Percent'])
                writer.writerows(csv_data)
            print(f"Data successfully saved to: {csv_filename}")
            print(f"Total data points saved: {len(csv_data)}")
        except Exception as e:
            print(f"Error saving CSV file: {e}")
    elif save_to_csv.get() and len(csv_data) == 0:
        print("No data to save - CSV logging was enabled but no data was collected.")
    else:
        print("CSV logging was disabled - no data saved.")
    
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

pid_frame = tk.Frame(root)
pid_frame.pack(pady=10)

# P Gain controls
p_frame = tk.Frame(pid_frame)
p_frame.pack(side=tk.LEFT, padx=10)
tk.Label(p_frame, text="P Gain").pack()
p_scale = tk.Scale(p_frame, from_=40.0, to=0.0, resolution=0.1, orient=tk.VERTICAL, command=send_p_gain)
p_scale.pack()
p_entry = tk.Entry(p_frame, width=8)
p_entry.pack(pady=5)

# I Gain controls
i_frame = tk.Frame(pid_frame)
i_frame.pack(side=tk.LEFT, padx=10)
tk.Label(i_frame, text="I Gain").pack()
i_scale = tk.Scale(i_frame, from_=900000.0, to=0.0, resolution=10.0, orient=tk.VERTICAL, command=send_i_gain)
i_scale.pack()
i_entry = tk.Entry(i_frame, width=8)
i_entry.pack(pady=5)

# D Gain controls
d_frame = tk.Frame(pid_frame)
d_frame.pack(side=tk.LEFT, padx=10)
tk.Label(d_frame, text="D Gain").pack()
d_scale = tk.Scale(d_frame, from_=10.0, to=0.0, resolution=0.1, orient=tk.VERTICAL, command=send_d_gain)
d_scale.pack()
d_entry = tk.Entry(d_frame, width=8)
d_entry.pack(pady=5)

# Minimum PWM controls
mp_frame = tk.Frame(pid_frame)
mp_frame.pack(side=tk.LEFT, padx=10)
tk.Label(mp_frame, text="Min PWM %").pack()
mp_scale = tk.Scale(mp_frame, from_=100.0, to=0.0, resolution=0.1, orient=tk.VERTICAL, command=send_min_pwm)
mp_scale.pack()
mp_entry = tk.Entry(mp_frame, width=8)
mp_entry.pack(pady=5)

# Send PID button
send_button_frame = tk.Frame(root)
send_button_frame.pack(pady=10)

send_button = tk.Button(send_button_frame, text="Send PID Parameters", command=send_pid_button_click, 
                       bg="lightgreen", font=("Arial", 12, "bold"), padx=20, pady=5)
send_button.pack(side=tk.LEFT, padx=5)

get_button = tk.Button(send_button_frame, text="Get PID Values", command=get_pid_values, 
                      bg="lightblue", font=("Arial", 12, "bold"), padx=20, pady=5)
get_button.pack(side=tk.LEFT, padx=5)

# CSV saving checkbox
csv_frame = tk.Frame(root)
csv_frame.pack(pady=5)

csv_checkbox = tk.Checkbutton(csv_frame, text="Save data to CSV file", variable=save_to_csv, 
                             font=("Arial", 10))
csv_checkbox.pack()

# Error information display
error_frame = tk.Frame(root)
error_frame.pack(pady=10, fill=tk.X)

error_title = tk.Label(error_frame, text="I2C Error Information", font=("Arial", 12, "bold"))
error_title.pack()

error_left_frame = tk.Frame(error_frame)
error_left_frame.pack(side=tk.LEFT, padx=20)
error_right_frame = tk.Frame(error_frame)
error_right_frame.pack(side=tk.LEFT, padx=20)

total_label = tk.Label(error_left_frame, text="Total Errors: 0", font=("Arial", 10))
total_label.pack(anchor=tk.W)
last_error_label = tk.Label(error_left_frame, text="Last Error: None", font=("Arial", 10))
last_error_label.pack(anchor=tk.W)
error_code_label = tk.Label(error_left_frame, text="Last Code: None", font=("Arial", 10))
error_code_label.pack(anchor=tk.W)

read_timeout_label = tk.Label(error_right_frame, text="Read Timeouts: 0", font=("Arial", 10))
read_timeout_label.pack(anchor=tk.W)
write_timeout_label = tk.Label(error_right_frame, text="Write Timeouts: 0", font=("Arial", 10))
write_timeout_label.pack(anchor=tk.W)

read_error_label = tk.Label(error_right_frame, text="Read Errors: 0", font=("Arial", 10))
read_error_label.pack(anchor=tk.W)
write_error_label = tk.Label(error_right_frame, text="Write Errors: 0", font=("Arial", 10))
write_error_label.pack(anchor=tk.W)
sensor_error_label = tk.Label(error_right_frame, text="Sensor Failures: 0", font=("Arial", 10))
sensor_error_label.pack(anchor=tk.W)

p_entry.bind('<Return>', on_p_entry_change)
p_entry.bind('<FocusOut>', on_p_entry_change)
i_entry.bind('<Return>', on_i_entry_change)
i_entry.bind('<FocusOut>', on_i_entry_change)
d_entry.bind('<Return>', on_d_entry_change)
d_entry.bind('<FocusOut>', on_d_entry_change)
mp_entry.bind('<Return>', on_mp_entry_change)
mp_entry.bind('<FocusOut>', on_mp_entry_change)

p_entry.insert(0, "2.5")
i_entry.insert(0, "900000.0")
d_entry.insert(0, "0.0")
mp_entry.insert(0, "0.0")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack()

ani = animation.FuncAnimation(fig, update, blit=True, interval=50, cache_frame_data=False)

root.mainloop()
