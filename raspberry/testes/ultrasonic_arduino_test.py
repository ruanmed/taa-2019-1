import serial
import re

ser = serial.Serial('/dev/ttyUSB0', 9600)
#ser.write('5')
#ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X
#ser.read()

def read_sonar():
	S = [None,None,None,None]

	while (S[0] == None or S[1] == None or S[2] == None or S[3] == None):
		V_SERIAL = ser.readline()
		valores = re.findall('[0-9]+', str(V_SERIAL))
		S[int(valores[0])] = int(valores[1])

	return S

while(True):

#	VALUE_SERIAL = ser.readline()

#	print(f'Retorno da porta serial: {VALUE_SERIAL}')

	new = read_sonar()

	print(new)
