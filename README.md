# ESP32 BLDCMotor & Servo Control

This folder contains the firmware and python test script for controlling the smart sorting bin via MQTT on an ESP32.

## MQTT Configuration

- **Broker**: `broker.emqx.io`
- **Port**: `1883`

## MQTT Topics

The ESP32 subscribes to and publishes on the following MQTT topics:

### 1. Status `imperial/yh4222/esp32/status`
- **Direction**: `ESP32 -> Broker` (Publish)
- **Description**: The ESP32 periodically sends status updates and execution completion events.
- **Example Payload**: `STATE,target=0.000,shaft=0.051,return=0,tilt=0,scur=112.0,stgt=112.0,sspd=100.0` or `EXEC_COMPLETED`

### 2. Calibration `imperial/yh4222/esp32/calib`
- **Direction**: `Broker -> ESP32` (Subscribe)
- **Description**: Sends a calibration value to the ESP32. Set plate zero angle.

### 3. Resolution/Bin selection `imperial/yh4222/esp32/res`
- **Direction**: `Broker -> ESP32` (Subscribe)
- **Description**: Commands the ESP32 to drop into a specific bin `1`, `2`, `3`, or `4`.
  - `1`: 0 degrees offset
  - `2`: -90 degrees offset (-PI/2)
  - `3`: -180 degrees offset (-PI)
  - `4`: 90 degrees offset (PI/2)
- **Example Payload**: `1`

### 4. Test `imperial/yh4222/esp32/test`
- **Direction**: `Broker -> ESP32` (Subscribe)

## Files

- **`BLDCFOC.ino`**: The main Arduino sketch running on the ESP32. Implements SimpleFOC, MQTT connection, Motor, and Servo control logic.
- **`main.py`**: A simple python script listening to all these MQTT topics to track and debug messages.
