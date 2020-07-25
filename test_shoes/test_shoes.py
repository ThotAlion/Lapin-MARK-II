import time,json
from serial import Serial
PS = Serial('/dev/ttyAMA0',115200,timeout=0.1)
PS.flushInput()
info = {}

while True:
    PS.write(b"A")
    out = PS.readline()
    try:
        info = json.loads(out)
    except:
        pass
    print(info['GYR'])
    time.sleep(0.01)