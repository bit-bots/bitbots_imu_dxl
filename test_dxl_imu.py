import serial
from binascii import hexlify
from time import sleep


def read_dxl_status_pkg(ser):
    header = ser.read(4)
    if header != b'\xFF\xFF\xFD\x00':
        return False
    dxl_id = ser.read()
    length = int.from_bytes(ser.read(2), 'little')
    data = ser.read(length)
    if len(data) != length:
        return False
    return True
    

ser = serial.Serial('/dev/ttyUSB2', 1_000_000, timeout=1)

read_imu = b'\xFF\xFF\xFD\x00\xF1\x07\x00\x02\x24\x00\x28\x00\x2F\x7F'
sync_read = b'\xFF\xFF\xFD\x00\xFE\x09\x00\x82\x7E\x00\x0a\x00\x13\x14\x4F\x7E'
response_19 = b'\xFF\xFF\xFD\x00\x13\x0E\x00\x55\x00\xF5\xFF\xFF\xFF\xFF\xFF\x05\x08\x00\x00\x39\x6A'
response_20 = b'\xFF\xFF\xFD\x00\x14\x0F\x00\x55\x00\x25\x00\xF6\xFF\xFF\xFF\xFD\xFD\x07\x00\x00\xE4\x3B'

print('Testing simple send/recv... ', end='')
n_bytes = ser.write(read_imu)
if read_dxl_status_pkg(ser):
    print('Success!')
else:
    print('Failure!')
    #exit()

#print('Testing response to packets with byte stuffing... ', end='')
#ser.write(sync_read)
#sleep(0.02)
#ser.write(response_19)
#sleep(0.02)
#ser.write(response_20)
#sleep(0.02)
while True:
    ser.write(read_imu)
    sleep(1.0)

if read_dxl_status_pkg(ser):
    print('Success!')
else:
    print('Failure!')
