import serial, glob, time

ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
print(f"Puertos encontrados: {ports}")

for port in ports:
    print(f"\nEscuchando {port} por 5 segundos...")
    with serial.Serial(port, 115200, timeout=0.58) as s:
        t0 = time.time()
        while time.time() - t0 < 5:
            line = s.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  → {line}")
                