from hardware.port_controller import PortController

# A wrapper class to communicate with the arduino motor controller

# TODO consider output of arduino; events maybe?
# this is a good idea; but probably not at this (the 2024) competition.
# if future people are looking at this, use asyncio to make an event system


class MotorSerial:
    def __init__(self, port):

        self.port = port
        self.serial = PortController(port)

    def send(self, commands, debug=False):
        """
        Send a packet of commands over this port.\n
        Commands is a dictionary in the form of {pin: magnitude}
        """

        packets = []

        for pin, magnitude in commands.items():
            packets.append(f"{magnitude},{pin}:")
        
        packet_string = ''.join(packets)
        self.serial.write(packet_string)
        if debug:
            print(packet_string)

    def kill(self):
        commands = {}
        for i in range(20): # arbitrary lol
            commands[i] = 0
        self.send(commands)