import time
import serial
import numpy as np
import soundcard as sc
import random
import warnings

warnings.filterwarnings("ignore")

# --- CONFIGURATION ---
COM_PORT = 'COM20'       
BAUD_RATE = 115200
BLOCK_SIZE = 1024        
SAMPLE_RATE = 48000      

MODE_SETTINGS = {
    "1": {"name": "Extremely Jumpy", "decay": 0.2, "blink_prob": 0.7},
    "2": {"name": "Jumpy",           "decay": 0.5, "blink_prob": 0.4},
    "3": {"name": "Normal",          "decay": 0.7, "blink_prob": 0.2},
    "4": {"name": "Calm",            "decay": 0.9, "blink_prob": 0.05}
}

def select_mode():
    print("\n--- Dashboard Mode Selection ---")
    print("1: Extremely Jumpy")
    print("2: Jumpy")
    print("3: Normal")
    print("4: Calm")
    choice = input("Select a mode (1-4): ")
    return MODE_SETTINGS.get(choice, MODE_SETTINGS["3"])

def main():
    selected = select_mode()
    DECAY_RATE = selected["decay"]
    BLINK_PROB = selected["blink_prob"]
    print(f"\n🚀 Running in '{selected['name']}' mode...")

    try:
        mic = [m for m in sc.all_microphones(include_loopback=True) if "Realtek" in m.name][0]
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0)
    except Exception as e:
        print(f"❌ Connection Error: {e}")
        return

    freq_bins = np.fft.rfftfreq(BLOCK_SIZE, d=1.0/SAMPLE_RATE)
    bass_mask = (freq_bins >= 40) & (freq_bins <= 200)
    mid_high_mask = (freq_bins >= 500) & (freq_bins <= 4000)

    last_speed = 0
    last_rpm = 0

    try:
        with mic.recorder(samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE) as recorder:
            while True:
                try:
                    data = recorder.record(numframes=BLOCK_SIZE)
                except:
                    continue

                fft_mag = np.abs(np.fft.rfft(np.mean(data, axis=1)))

                raw_bass = np.mean(fft_mag[bass_mask]) if np.any(bass_mask) else 0
                raw_rpm = np.mean(fft_mag[mid_high_mask]) if np.any(mid_high_mask) else 0

                target_speed = int(np.clip(raw_bass * 18.0, 0, 255))
                target_rpm = int(np.clip(raw_rpm * 18.0, 0, 255))

                last_speed = target_speed if target_speed > last_speed else int(last_speed * DECAY_RATE)
                last_rpm = target_rpm if target_rpm > last_rpm else int(last_rpm * DECAY_RATE)

                blink_state = 1 if random.random() < BLINK_PROB else 0

                payload = f"{last_rpm},{last_speed},{blink_state}\n"
                ser.write(payload.encode())
                
                time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()