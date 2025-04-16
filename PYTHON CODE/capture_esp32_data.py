import serial
import pandas as pd
from datetime import datetime
import time
import os

# Configuration
SERIAL_PORT = 'COM3'  # Update to your ESP32 port (e.g., 'COM3')
BAUD_RATE = 115200
OUTPUT_DIR = 'esp32_data'  # Directory to save CSVs
TIMEOUT = 1  # Serial read timeout in seconds

def ensure_output_dir(directory):
    """Create output directory if it doesn't exist."""
    if not os.path.exists(directory):
        os.makedirs(directory)

def get_timestamped_filename():
    """Generate a timestamped filename for the CSV."""
    return f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def capture_data():
    """Capture data from ESP32 and save as CSV for each session."""
    try:
        # Initialize serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        # Create output directory
        ensure_output_dir(OUTPUT_DIR)

        print("Waiting for first session... (Reset ESP32 to start)")

        while True:  # Run until Ctrl+C
            # Prepare for a new session
            data = []
            header_received = False
            session_active = False

            while True:  # Process one session
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue  # Skip empty lines

                # Check for session start (header)
                if line.startswith("timestamp,accX,accY,accZ"):
                    if not header_received:
                        header_received = True
                        session_active = True
                        print(f"New session started. Collecting samples...")
                        continue

                # Check for session end
                if "Session complete" in line and session_active:
                    print("Session complete.")
                    break

                # Process data lines
                if header_received and session_active:
                    try:
                        values = line.split(',')
                        if len(values) == 4:
                            timestamp = float(values[0])
                            accX = float(values[1])
                            accY = float(values[2])
                            accZ = float(values[3])
                            data.append([timestamp, accX, accY, accZ])
                            if len(data) % 100 == 0:
                                print(f"Collected {len(data)} samples...")
                    except ValueError:
                        print(f"Skipping invalid line: {line}")
                        continue

            # Save data to CSV if collected
            if data:
                df = pd.DataFrame(data, columns=['timestamp', 'accX', 'accY', 'accZ'])
                filename = os.path.join(OUTPUT_DIR, get_timestamped_filename())
                df.to_csv(filename, index=False)
                print(f"Saved {len(df)} samples to {filename}")
            else:
                print("No data collected in this session.")

            print("Waiting for next session... (Reset ESP32 again)")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
        if 'ser' in locals():
            ser.close()
    except Exception as e:
        print(f"Unexpected error: {e}")
        if 'ser' in locals():
            ser.close()
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    capture_data()