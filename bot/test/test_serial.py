import serial

ser_id = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_858303036393512071F0-if00'
with serial.Serial(ser_id, 115200) as ser:
	while True:
		while True:
			res = ser.readline()
			if res:
				res_dec = res.decode().strip()
				print(f"Response: {res_dec}")
				if "OK" in res_dec:
					break
			else:
				print("No response from the robot.")
				break
		x = input("cmd > ")
		if x == "exit":
			break
		ser.write((x + '\n').encode())
		