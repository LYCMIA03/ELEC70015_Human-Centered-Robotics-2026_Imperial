import paho.mqtt.client as mqtt

# --- MQTT Configuration ---
BROKER = "broker.emqx.io"
PORT = 1883
CLIENT_ID = "python_monitor_script" # Client IDs must be unique

# Topics to monitor (Topic, QoS level)
# Alternatively, you could just subscribe to "imperial/yh4222/esp32/#" to get everything under that path.
TOPICS = [
    ("imperial/yh4222/esp32/test", 0),
    ("imperial/yh4222/esp32/calib", 0),
    ("imperial/yh4222/esp32/res", 0),
    ("imperial/yh4222/esp32/status", 0)
]

# --- Callbacks ---
def on_connect(client, userdata, flags, reason_code, properties=None):
    """Callback triggered when the client connects to the broker."""
    if reason_code == 0:
        print(f"Successfully connected to {BROKER}:{PORT}")
        # Subscribe to the predefined topics
        client.subscribe(TOPICS)
        print("Listening for messages on:")
        for topic, qos in TOPICS:
            print(f"  -> {topic}")
        print("-" * 40)
    else:
        print(f"Failed to connect. Reason code: {reason_code}")

def on_message(client, userdata, msg):
    """Callback triggered when a message is received on a subscribed topic."""
    # Attempt to decode the payload as a UTF-8 string
    try:
        payload = msg.payload.decode('utf-8')
    except UnicodeDecodeError:
        payload = msg.payload # Fallback to raw bytes if it's binary data
        
    print(f"[Topic]: {msg.topic}")
    print(f"[Payload]: {payload}\n")

# --- Main Execution ---
if __name__ == "__main__":
    # Initialize the MQTT Client (Using VERSION2 API for newer paho-mqtt support)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)

    # Attach the callbacks to the client
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the broker
    print(f"Connecting to {BROKER}...")
    try:
        client.connect(BROKER, PORT, keepalive=60)
        
        # loop_forever() blocks the program and handles all network traffic and reconnecting
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\nDisconnecting from broker...")
        client.disconnect()
        print("Monitor stopped.")
    except Exception as e:
        print(f"An error occurred: {e}")