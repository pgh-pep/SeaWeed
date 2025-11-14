import can
import time 

bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=500000)
while True:
    # Send STOP 
    message = can.Message(
        arbitration_id=0x0E1,
        data=[],
        dlc=0
    )
    # Send the message
    bus.send(message)
    time.sleep(5000)
    message = can.Message(
        arbitration_id=0x0E2,
        data=[],
        dlc=0
    )
    time.sleep(5000)
    bus.send(message)



# Receive a message (optional)
# received_message = bus.recv(timeout=1.0)

# Shut down the bus
bus.shutdown()