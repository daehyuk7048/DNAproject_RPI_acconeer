# rpi_bridge.py
import socket, serial, time

HOST, PORT = "0.0.0.0", 9999
SERIAL_PORT, BAUD_RATE = "/dev/ttyACM0", 9600

def handle_client(conn, ser):
    buf = b""
    conn.sendall(b"READY\n")
    while True:
        data = conn.recv(1024)
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            # 예: b'P11,75.5'
            cmd = line.decode("utf-8", errors="ignore")
            ser.write((cmd + "\n").encode("utf-8"))
            # 선택: 응답 한 줄만 돌려보내기
            try:
                ser.timeout = 0.2
                resp = ser.readline().strip()
                if resp:
                    conn.sendall(resp + b"\n")
            except Exception:
                pass

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"[BRIDGE] Arduino {SERIAL_PORT}@{BAUD_RATE}")
    except serial.SerialException as e:
        print(f"[ERR] Serial open fail: {e}")
        return

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"[BRIDGE] Listening on {HOST}:{PORT}")
        while True:
            conn, addr = s.accept()
            print(f"[BRIDGE] Client {addr}")
            try:
                handle_client(conn, ser)
            except ConnectionResetError:
                print("[BRIDGE] Client reset")
            except Exception as e:
                print(f"[BRIDGE] Error: {e}")
            finally:
                conn.close()

if __name__ == "__main__":
    main()
