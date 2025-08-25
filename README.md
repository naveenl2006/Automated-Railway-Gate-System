



# 🚦 Automatic Railway![Project Image](https://github.com/user-attachments/assets/65f25ec9-d187-41c9-b71d-f7da10171a09)


https://github.com/user-attachments/assets/7a95f39a-8943-4baf-825a-653283a9476a

 Gate Control System  

[![Arduino](https://img.shields.io/badge/Made%20with-Arduino-blue?logo=arduino)](https://www.arduino.cc/)  
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)  
[![Status](https://img.shields.io/badge/Project-Stable-success)]()  

An **Arduino-based Automatic Railway Gate Control System** using **dual IR sensors** and **dual servo motors** for enhanced safety and automation. The system automatically detects an approaching train, closes the gates, and reopens them once the train has cleared.  

---

## 🛠 Components Required  

- Arduino UNO / Nano  
- 2x IR Sensors  
- 2x Servo Motors (SG90 or MG90S)  
- 2x Red LEDs  
- 2x Green LEDs  
- Buzzer (3-pin active/passive)  
- Jumper Wires, Resistors, Breadboard  
- 5V Power Supply  

---

## ⚙️ Features  

✔️ Dual IR Sensor Detection – train sensed from both sides  
✔️ Dual Servo Gates – left & right gates move together  
✔️ Emergency Gate Closure – fast servo response for safety  
✔️ Safe Gate Opening – moderate reopening for pedestrians/vehicles  
✔️ Visual Indication – LEDs show gate status  
✔️ Audible Warning – buzzer alerts during train passage  
✔️ Train Direction Awareness – logs whether train came from Side A, Side B, or both  

---

## 🔌 Pin Connections  

| Component         | Arduino Pin | Description |
|------------------|-------------|-------------|
| IR Sensor 1      | D2          | Train detection (Side A) |
| IR Sensor 2      | D3          | Train detection (Side B) |
| Red LED 1        | D4          | Left warning light |
| Red LED 2        | D5          | Right warning light |
| Green LED 1      | D6          | Left safe light |
| Green LED 2      | D7          | Right safe light |
| Buzzer (Signal)  | D8          | Audible warning |
| Servo Motor 1    | D9 (PWM)    | Left gate servo |
| Servo Motor 2    | D10 (PWM)   | Right gate servo |

---

## 🚧 Gate Logic  

- **Normal State (No Train Detected)**  
  ✅ Green LEDs ON  
  ❌ Red LEDs OFF  
  🔇 Buzzer OFF  
  🟢 Gates OPEN (0°)  

- **Train Detected (Any Sensor Active)**  
  ❌ Green LEDs OFF  
  ✅ Red LEDs ON  
  🔊 Buzzer ON (warning pattern)  
  🔒 Gates CLOSED (90°)  

- **Train Cleared (Both Sensors Inactive)**  
  ⏳ After 3s delay → Gates reopen safely  
  ✅ Green LEDs ON, ❌ Red LEDs OFF  
  🔇 Buzzer OFF  

---

## 📂 File Structure  

