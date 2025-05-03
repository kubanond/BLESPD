import socket
import csv
import datetime
#36 kroku

UDP_IP = "0.0.0.0"
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening for UDP broadcasts on port {UDP_PORT}...")

csv_filename = "sensor_data.csv"

try:
    with open(csv_filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "X", "Y", "Z"])
        print(f"Created new CSV file: {csv_filename}")
except FileExistsError:
    print(f"Appending to existing CSV file: {csv_filename}")

with open(csv_filename, 'a', newline='') as csvfile:
    writer = csv.writer(csvfile)

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            print(f"Received from {addr}: {message}")

            parts = message.split(',')
            if len(parts) < 3:
                print("Unexpected message format, skipping.")
                continue

            x = int(parts[0].split(':')[1].strip())
            y = int(parts[1].split(':')[1].strip())
            z = int(parts[2].split(':')[1].strip())

            timestamp = datetime.datetime.now().isoformat()

            writer.writerow([timestamp, x, y, z])
            csvfile.flush()

        except Exception as e:
            print("Error processing message:", e)


if __name__ == "__main__":
    pass