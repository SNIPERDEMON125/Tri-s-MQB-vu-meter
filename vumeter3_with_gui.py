import time
import serial
import numpy as np
import soundcard as sc
import random
import warnings
import threading
import tkinter as tk
import traceback

# Suppress warnings
warnings.filterwarnings("ignore")

# --- CONFIGURATION ---
COM_PORT = 'COM20'
BAUD_RATE = 115200
BLOCK_SIZE = 1024
SAMPLE_RATE = 48000
NUM_BARS = 24
GUI_UPDATE_MS = 30

# Shared state
lock = threading.Lock()
latest_bars = np.zeros(NUM_BARS)
latest_rpm, latest_speed, latest_blink = 0, 0, 0
running = True

def get_device_index():
    devices = sc.all_microphones(include_loopback=True)
    print("\n--- Available Audio Devices ---")
    for i, dev in enumerate(devices):
        print(f"[{i}] {dev.name}")
    try:
        return int(input("\nEnter the index [number] of your Loopback device: "))
    except:
        return 0

def audio_loop(selected, dev_idx):
    global latest_bars, latest_rpm, latest_speed, latest_blink, running
    
    # Robust initialization with error reporting
    try:
        devices = sc.all_microphones(include_loopback=True)
        mic = devices[dev_idx]
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0)
        print(f"\nDEBUG: Successfully connected to {mic.name} and {COM_PORT}")
    except Exception as e:
        print(f"\n--- FATAL ERROR ---")
        traceback.print_exc()
        input("\nPress Enter to close this window...")
        running = False
        return

    freq_bins = np.fft.rfftfreq(BLOCK_SIZE, d=1.0 / SAMPLE_RATE)
    bass_mask = (freq_bins >= 40) & (freq_bins <= 200)
    mid_high_mask = (freq_bins >= 500) & (freq_bins <= 4000)
    bar_edges = np.logspace(np.log10(40), np.log10(16000), NUM_BARS + 1)

    last_speed, last_rpm = 0, 0

    try:
        with mic.recorder(samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE) as recorder:
            while running:
                data = recorder.record(numframes=BLOCK_SIZE)
                fft_mag = np.abs(np.fft.rfft(np.mean(data, axis=1)))

                raw_bass = np.mean(fft_mag[bass_mask]) if np.any(bass_mask) else 0
                raw_rpm = np.max(fft_mag[mid_high_mask]) if np.any(mid_high_mask) else 0

                target_speed = int(np.clip(raw_bass * 18.0, 0, 255))
                target_rpm = int(np.clip(raw_rpm * 20.0, 0, 255))

                last_speed = target_speed if target_speed > last_speed else int(last_speed * selected["decay"])
                last_rpm = target_rpm if target_rpm > last_rpm else int(last_rpm * 0.3)

                blink = 1 if random.random() < selected["blink_prob"] else 0
                ser.write(f"{last_rpm},{last_speed},{blink}\n".encode())

                bars = np.zeros(NUM_BARS)
                for i in range(NUM_BARS):
                    mask = (freq_bins >= bar_edges[i]) & (freq_bins < bar_edges[i+1])
                    if np.any(mask): bars[i] = fft_mag[mask].mean()
                
                with lock:
                    latest_bars = np.clip(bars / 8.0, 0, 1)
                    latest_rpm, latest_speed, latest_blink = last_rpm, last_speed, blink
    except Exception as e:
        print(f"\n--- LOOP ERROR ---")
        traceback.print_exc()
    finally:
        ser.close()
        running = False

class App(tk.Tk):
    def __init__(self, dev_idx, mode):
        super().__init__()
        self.title("Dash Control"); self.geometry("640x360"); self.configure(bg="#101010")
        self.canvas = tk.Canvas(self, width=620, height=260, bg="#101010", highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)
        self.label = tk.Label(self, text="RPM: -- | Speed: --", bg="#101010", fg="white", font=("Consolas", 12))
        self.label.pack()
        self.bars = [self.canvas.create_rectangle(i*25+2, 260, i*25+23, 260, fill="#00ffaa", outline="") for i in range(NUM_BARS)]
        
        threading.Thread(target=audio_loop, args=(mode, dev_idx), daemon=True).start()
        self.after(30, self.update_gui)

    def update_gui(self):
        with lock: bars, rpm, speed = latest_bars.copy(), latest_rpm, latest_speed
        for i, rect in enumerate(self.bars):
            h = bars[i] * 260
            self.canvas.coords(rect, i*25+2, 260-h, i*25+23, 260)
            self.canvas.itemconfig(rect, fill="#ff3300" if bars[i] > 0.8 else "#00ffaa")
        self.label.config(text=f"RPM: {rpm} | Speed: {speed}")
        if running: self.after(30, self.update_gui)
        else: self.destroy()

if __name__ == '__main__':
    dev_idx = get_device_index()
    print("1: Ex. Jumpy | 2: Jumpy | 3: Normal | 4: Calm")
    choice = input("Mode: ")
    mode = {"1":{"decay":0.2,"blink_prob":0.7},"2":{"decay":0.5,"blink_prob":0.4},"3":{"decay":0.7,"blink_prob":0.2},"4":{"decay":0.9,"blink_prob":0.05}}[choice]
    
    App(dev_idx, mode).mainloop()
    running = False
