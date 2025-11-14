import can
class CanInterface: 
    def __init__(self):
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0', receive_own_messages=True)
            print("CAN bus initialized successfully.")
        except Exception as e:
            print(f"Error initializing CAN bus: {e}")
            exit()

    
    def send(data, id):
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)

    
    
    