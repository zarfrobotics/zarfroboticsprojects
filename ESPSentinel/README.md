ESPCam — ESP32-CAM Security System
Full-featured AI Thinker ESP32-CAM firmware + modern web dashboard  
Built with ❤️ by Zarf Robotics
---
✨ Overview
ESPCam is a complete open-source wireless security camera system based on the AI Thinker ESP32-CAM. It features a beautiful, responsive web dashboard with live streaming, PTZ control, motion detection, SD card recording, and full remote management.
Perfect for home security, workshop monitoring, wildlife observation, or robotics projects.
---
🚀 Features
Camera & Streaming
30 FPS MJPEG live stream (Port 81)
VGA (640×480) resolution with adjustable quality
Night mode, brightness, contrast, saturation, flip controls
Digital zoom & rotation in browser
Recording & Storage
15 FPS AVI recording to SD card (1-minute segments)
Automatic folder organization by date/time (`/videos/YYYY\_MM\_DD/HH00-HH00/`)
Snapshot capture (saved to SD + downloadable)
Full SD Card Browser — list, download, and delete videos & photos
Motion Detection
PIR sensor (HC-SR501) support on GPIO2
Automatic snapshot on motion
Real-time motion banner and event logging
Pan/Tilt Control (PTZ)
Support for two SG90/MG90S servos
On-screen D-Pad + fine adjustment sliders
Center function
Additional Controls
PWM controlled LED flashlight (GPIO4)
Horizontal & Vertical flip
Pause / Resume stream
Factory reset from browser
Web Dashboard
Modern cyber-tech dark UI with glowing accents
Fully responsive (optimized for landscape mobile)
Real-time stats: FPS, Chip Temperature, RSSI, Uptime, SD usage
Live clock and connection status
---
🛠 Hardware Requirements
Component	Details
Board	AI Thinker ESP32-CAM
SD Card	MicroSD (FAT32, max 32GB recommended)
Servos	2× SG90 or MG90S (Pan + Tilt)
PIR Sensor	HC-SR501 (optional)
Power Supply	5V, minimum 2A recommended
Pin Configuration
Function	GPIO	Notes
Pan Servo	12	Attached after SD init
Tilt Servo	13	Attached after SD init
PIR Sensor	2	Motion detection
LED Flash	4	PWM brightness control
> \*\*Important\*\*: GPIO12 is a strapping pin. Servos are attached \*\*after\*\* SD card initialization to avoid boot issues.
---
📥 Installation
Flash the Firmware
Open `ESPCam.ino` in Arduino IDE
Board: AI Thinker ESP32-CAM
Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
Flash speed: 921600
Open the Web Dashboard
Open the `ESPCam.html` from the downloaded folder and open it in any web browser
Insert a formatted SD card
Power on the ESP32-CAM
---
⚙ Configuration
Edit these lines at the top of `ESPCam.ino`:
```cpp
#define WIFI\_SSID    "Your\_WiFi\_Name"
#define WIFI\_PASS    "Your\_WiFi\_Password"
#define DEVICE\_NAME  "ESPCam"
```
---
🎮 Using the Web Dashboard
After booting, open the ESPCam HTML file.
On first launch, enter your name and the camera IP address.
Main Controls:
📸 Snapshot — Save image to SD card
⬇ Download — Save current frame to your device
🗂 Browse SD — Full file manager for videos & photos
PTZ Tab — Pan/Tilt control
Camera Tab — Image quality settings
Motion Tab — PIR status and event log
---
🔌 API Endpoints (Port 82)
Endpoint	Description
`/stream`	MJPEG Live Stream (Port 81)
`/snapshot`	Download current JPEG
`/savesnap`	Save snapshot to SD
`/status`	System status (JSON)
`/servo?pan=X\&tilt=Y`	Control servos
`/led?state=0-255`	LED brightness
`/camset?quality=10\&...`	Camera settings
`/sd/list?path=/videos`	List files/folders
`/sd/download?path=...`	Download file
`/sd/delete?path=...`	Delete file
`/pir`	Current PIR motion status
