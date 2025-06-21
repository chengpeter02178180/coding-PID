# coding-PID
#使用前先下載VISUAL CODE
#打開程式新增一個Folder
#在Folder裡下存入pid_simulator_fixed_final檔案
#開啟pid_simulator_fixed_final存入下列程式碼
#存入之後在終端機(terimal)打上pip install matplotlib fpdf numpy
#安裝完之後再次在終端機(terimal)打上python pid_simulator_fixed_final.py
#如果終端機打上python pid_simulator_fixed_final.py無法執行時，請打python .\pid_simulator_fixed_final.py
#安裝包網址:https://drive.google.com/file/d/1TD9-LO3fg-ZfZtbLuYVwdOlehUvzuLJ_/view?usp=drive_link

#PID模擬器設計程式(完整複製以下程式)
import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
import csv
import random
from collections import deque
from fpdf import FPDF
import numpy as np

matplotlib.rcParams['font.family'] = 'Microsoft JhengHei'
matplotlib.rcParams['axes.unicode_minus'] = False

LANGUAGES = {
    "zh": {
        "title": "PID 控制模擬器（動畫+自動調參）",
        "kp": "比例參數 Kp",
        "ki": "積分參數 Ki",
        "kd": "微分參數 Kd",
        "setpoint": "目標值 Setpoint",
        "model": "系統模型",
        "simulate": "動畫模擬播放",
        "autotune": "自動最佳化 PID",
        "noise": "加入雜訊",
        "delay": "加入延遲",
        "friction": "加入摩擦",
        "export_csv": "匯出誤差與輸出",
        "export_pdf": "匯出模擬報告 PDF",
        "language": "語言切換"
    }
}

current_lang = "zh"

def switch_language():
    apply_language()

def apply_language():
    lang = LANGUAGES[current_lang]
    root.title(lang["title"])
    label_kp.config(text=lang["kp"])
    label_ki.config(text=lang["ki"])
    label_kd.config(text=lang["kd"])
    label_setpoint.config(text=lang["setpoint"])
    label_model.config(text=lang["model"])
    simulate_btn.config(text=lang["simulate"])
    tune_btn.config(text=lang["autotune"])
    noise_cb.config(text=lang["noise"])
    delay_cb.config(text=lang["delay"])
    friction_cb.config(text=lang["friction"])
    export_csv_btn.config(text=lang["export_csv"])
    export_pdf_btn.config(text=lang["export_pdf"])

model_var = 'acceleration'

def update_model(selection):
    global model_var
    if selection == "加速度模型":
        model_var = 'acceleration'
    else:
        model_var = 'first_order'

def simulate_pid(Kp, Ki, Kd, setpoint, noise=False, delay=False, friction=False, model='acceleration'):
    dt = 0.1
    total_time = 20
    steps = int(total_time / dt)
    process_value = 0
    velocity = 0
    error_sum = 0
    last_error = 0
    delay_queue = deque([0]*5) if delay else None

    time_list = []
    value_list = []
    error_list = []
    output_list = []

    for i in range(steps):
        measured_value = process_value + random.uniform(-0.5, 0.5) if noise else process_value
        error = setpoint - measured_value
        error_sum += error * dt
        d_error = (error - last_error) / dt
        output = Kp * error + Ki * error_sum + Kd * d_error
        last_error = error

        if delay:
            delay_queue.append(output)
            output = delay_queue.popleft()

        friction_force = 0.1 * velocity if friction else 0

        if model == 'acceleration':
            velocity += (output - friction_force) * dt
            process_value += velocity * dt
        elif model == 'first_order':
            tau = 2.0
            process_value += (-(process_value - output) / tau) * dt

        time_list.append(i * dt)
        value_list.append(process_value)
        error_list.append(error)
        output_list.append(output)

    max_value = max(value_list)
    overshoot = ((max_value - setpoint) / setpoint) * 100 if setpoint != 0 else 0
    tolerance = 0.05 * setpoint
    settling_time = 0
    for i in range(len(value_list)-1, -1, -1):
        if abs(value_list[i] - setpoint) > tolerance:
            settling_time = time_list[i+1] if i+1 < len(time_list) else time_list[-1]
            break

    return time_list, value_list, overshoot, settling_time, error_list, output_list

def autotune_pid():
    setpoint = float(setpoint_entry.get())
    best_score = float('inf')
    best_k = (1.0, 0.01, 0.1)
    for _ in range(50):
        kp = random.uniform(0.1, 5.0)
        ki = random.uniform(0.0, 1.0)
        kd = random.uniform(0.0, 2.0)
        _, val, over, settle, _, _ = simulate_pid(kp, ki, kd, setpoint,
            noise=var_noise.get(), delay=var_delay.get(), friction=var_friction.get(), model=model_var)
        score = abs(over) + settle
        if score < best_score:
            best_score = score
            best_k = (kp, ki, kd)
    kp_scale.set(best_k[0])
    ki_scale.set(best_k[1])
    kd_scale.set(best_k[2])

def run_simulation():
    Kp = kp_scale.get()
    Ki = ki_scale.get()
    Kd = kd_scale.get()
    setpoint = float(setpoint_entry.get())
    time_list, value_list, overshoot, settling_time, error_list, output_list = simulate_pid(
        Kp, Ki, Kd, setpoint, var_noise.get(), var_delay.get(), var_friction.get(), model_var)
    ax.clear()
    ax.plot(time_list, value_list, label='系統輸出')
    ax.axhline(y=setpoint, color='r', linestyle='--', label='目標值')

    max_value = max(value_list)
    overshoot = ((max_value - setpoint) / setpoint) * 100 if setpoint != 0 else 0
    tolerance = 0.05 * setpoint
    settling_time = 0
    for i in range(len(value_list)-1, -1, -1):
        if abs(value_list[i] - setpoint) > tolerance:
            settling_time = time_list[i+1] if i+1 < len(time_list) else time_list[-1]
            break
    ax.axvline(x=settling_time, color='gray', linestyle=':', label=f'穩定時間 ≈ {settling_time:.1f}s')
    ax.annotate(f'超調量 {overshoot:.1f}%', xy=(time_list[value_list.index(max_value)], max_value),
                xytext=(10, max_value+10), arrowprops=dict(facecolor='black', arrowstyle='->'))

    ax.set_title('PID 控制模擬')
    ax.legend()
    ax.grid(True)
    canvas.draw()
    export_csv_btn.config(command=lambda: export_error_output(error_list, output_list, time_list))
    export_pdf_btn.config(command=lambda: export_pdf_report(Kp, Ki, Kd, setpoint, overshoot, settling_time, fig))

def export_error_output(error_list, output_list, time_list):
    file_path = filedialog.asksaveasfilename(defaultextension=".csv",
                                             filetypes=[("CSV files", "*.csv")])
    if file_path:
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["時間 (秒)", "誤差", "控制輸出"])
            for t, e, o in zip(time_list, error_list, output_list):
                writer.writerow([t, e, o])

def export_pdf_report(Kp, Ki, Kd, setpoint, overshoot, settling_time, fig):
    file_path = filedialog.asksaveasfilename(defaultextension=".pdf",
                                             filetypes=[("PDF files", "*.pdf")])
    if file_path:
        image_path = file_path.replace(".pdf", ".png")
        fig.savefig(image_path)
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, txt="PID 控制模擬報告", ln=True, align='C')
        pdf.ln(10)
        pdf.multi_cell(0, 10, txt=(
            f"Kp: {Kp}\nKi: {Ki}\nKd: {Kd}\n目標值: {setpoint}\n"
            f"超調量: {overshoot:.2f}%\n穩定時間: {settling_time:.2f} 秒"
        ))
        pdf.image(image_path, x=10, y=80, w=180)
        pdf.output(file_path)

root = tk.Tk()
root.geometry("960x640")
control_frame = ttk.Frame(root)
control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

label_kp = ttk.Label(control_frame)
label_kp.pack()
kp_scale = tk.Scale(control_frame, from_=0, to=5, resolution=0.1, orient=tk.HORIZONTAL)
kp_scale.set(1.0)
kp_scale.pack()

label_ki = ttk.Label(control_frame)
label_ki.pack()
ki_scale = tk.Scale(control_frame, from_=0, to=1, resolution=0.01, orient=tk.HORIZONTAL)
ki_scale.set(0.01)
ki_scale.pack()

label_kd = ttk.Label(control_frame)
label_kd.pack()
kd_scale = tk.Scale(control_frame, from_=0, to=2, resolution=0.1, orient=tk.HORIZONTAL)
kd_scale.set(0.5)
kd_scale.pack()

label_setpoint = ttk.Label(control_frame)
label_setpoint.pack()
setpoint_entry = ttk.Entry(control_frame)
setpoint_entry.insert(0, "10")
setpoint_entry.pack()

label_model = ttk.Label(control_frame)
label_model.pack()
model_menu = ttk.Combobox(control_frame, values=["加速度模型", "一階系統"])
model_menu.current(0)
model_menu.pack()
model_menu.bind("<<ComboboxSelected>>", lambda e: update_model(model_menu.get()))

simulate_btn = ttk.Button(control_frame, command=run_simulation)
simulate_btn.pack(pady=5)

tune_btn = ttk.Button(control_frame, text="Autotune", command=autotune_pid)
tune_btn.pack(pady=5)

var_noise = tk.BooleanVar()
var_delay = tk.BooleanVar()
var_friction = tk.BooleanVar()
noise_cb = ttk.Checkbutton(control_frame, variable=var_noise)
noise_cb.pack(anchor='w')
delay_cb = ttk.Checkbutton(control_frame, variable=var_delay)
delay_cb.pack(anchor='w')
friction_cb = ttk.Checkbutton(control_frame, variable=var_friction)
friction_cb.pack(anchor='w')

export_csv_btn = ttk.Button(control_frame)
export_csv_btn.pack(pady=5)
export_pdf_btn = ttk.Button(control_frame)
export_pdf_btn.pack(pady=5)

fig, ax = plt.subplots(figsize=(5.5, 4.5))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

apply_language()
root.mainloop()
