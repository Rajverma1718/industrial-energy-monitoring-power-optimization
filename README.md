# Industrial Energy Monitoring & Power Optimization System

This is an ESP32 project made in Wokwi. It measures the voltage and current of an industrial load and calculates power, energy usage, and estimated electricity cost.

The project also keeps a short history of power readings. If the load becomes unusually high, it shows an alert and recommends reducing the load.

## Main features

- Voltage and current monitoring
- Power, energy, and electricity-cost calculation
- Average and peak power readings
- Abnormal-consumption alert
- DHT22 temperature and humidity sensor
- MPU6050 vibration sensor
- OLED display
- LED and buzzer status indicators
- SD-card CSV logging
- Wi-Fi dashboard with a live power graph

## How to run it

1. Open the Wokwi project and start the simulation.
2. Move the voltage and current potentiometers to change the simulated load.
3. Press the PAGE button to change the OLED screen.
4. Open the dashboard link shown in the Serial Monitor.

## Project files

- `sketch.ino` — ESP32 program
- `diagram.json` — Wokwi circuit diagram
- `libraries.txt` — required libraries

The simulator uses a power factor of 0.92 and a tariff of ₹8.50 per kWh. The potentiometers are only simulated sensor inputs. Do not connect an ESP32 directly to mains electricity.
