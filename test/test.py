import serial, time
from pathlib import Path

# get a file from https://nmeagen.org
file = str(Path.joinpath(Path.home(), 'workspace/gps_tracker/test/output.nmea'))
if len(sys.argv)>1:
    file = sys.argv[1]

with serial.Serial('/dev/ttyUSB1', 115200, timeout=5) as ser:
    with open(file) as f_:
        while True:
            c = f_.read(1)
            if c == b'$': 
                time.sleep(.1)
                
            if not c:
                break
                
            ser.write(c)
