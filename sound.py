import soundcard as sc
for mic in sc.all_microphones(include_loopback=True):
    print(f"Device Name: {mic.name}")