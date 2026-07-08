Audio-Reactive Gauge Dashboard

A hobbyist project that converts real-time audio frequencies into physical needle movement using an Arduino and hobby servos.
Project Overview

This project uses a Python-based digital signal processing (DSP) pipeline to analyze audio in real-time. It extracts frequency data (RPM for mid-highs, Speed for bass) and sends these values over Serial to an Arduino, which drives two servos to display the data on physical analog gauges.
Hardware Setup

    Microcontroller: Arduino (e.g., Uno, Nano, or Mega).

    VW Golf 7 Gauge Cluster

    MCPC2515 Module

Software Prerequisites

    Python 3.x

    Arduino IDE

    Required Python Libraries:
    Bash

    pip install pyserial numpy soundcard

Usage

    Arduino: Upload the code found in arduino_sketch/ to your board. Ensure your Serial.begin baud rate matches the Python script (115200).

    Python: * Open dashboard_app.py.

        Set the COM_PORT variable to your correct port (e.g., 'COM20').

        Run the script: python dashboard_app.py.

        Select your audio loopback device from the terminal list.

Troubleshooting

    "Access Denied": Ensure the Arduino Serial Monitor is closed before running the Python script.

    Needles not moving: Verify the baud rate is set to 115200 in both the Arduino and Python files. Ensure a Common Ground exists between the servo power supply and the Arduino.
