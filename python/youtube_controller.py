import pyautogui
import serial
import argparse
import time
import logging
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL

class MyControllerMap:
    def __init__(self):
        self.button = {'R': 'D','L':'A','D':'S','U':'W'} # Fast forward (10 seg) pro Youtube
    
class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    def read_packet(self):
        packet = ''
        stop = False
        while not(stop):
            char = self.ser.read().decode()
            if char == '<':  # Início do pacote
                packet = '<'
            elif char == '>' and packet:  # Fim do pacote
                packet += '>'
                stop = True
            elif packet:
                packet += char
       
        return packet.strip('<>')
    
    def update(self):
        ## Sync protocol
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        currentVolumeLevelScalar = volume.GetMasterVolumeLevelScalar()
        packet = self.read_packet()
        buttons = packet.split(',')
        print(buttons)
        if(float(buttons[4])<1):
            volume.SetMasterVolumeLevelScalar(float(buttons[4]), None)
        else:
            volume.SetMasterVolumeLevelScalar(0.99, None)
        if buttons[0]=='0':
            logging.info("KEYUP LEFT")
            pyautogui.keyUp(self.mapping.button['L'])
        else:
            logging.info("KEYDOWN RIGHT")
            pyautogui.keyDown(self.mapping.button['L'])
        if buttons[1]=='0':
            logging.info("KEYUP DOWN")
            pyautogui.keyUp(self.mapping.button['D'])
        else:
            logging.info("KEYDOWN DOWN")
            pyautogui.keyDown(self.mapping.button['D'])
        if buttons[2]=='0':
            logging.info("KEYUP UP")
            pyautogui.keyUp(self.mapping.button['U'])
        else:
            logging.info("KEYDOWN UP")
            pyautogui.keyDown(self.mapping.button['U'])
        if buttons[3]=='0':
            logging.info("KEYUP RIGHT")
            pyautogui.keyUp(self.mapping.button['R'])
        else:
            logging.info("KEYDOWN RIGHT")
            pyautogui.keyDown(self.mapping.button['R'])
        # while self.incoming != b'X':
        #     self.incoming = self.ser.read()
        #     logging.debug("Received INCOMING: {}".format(self.incoming))
        # for button in buttons:
        #     print(button)
        # data = self.ser.read()
        # logging.debug("Received DATA: {}".format(data))
        # print(data)
        # if data == b'XL':
        #     logging.info("KEYDOWN A")
        #     pyautogui.keyDown(self.mapping.button['B'])
        # elif data == b'L':
        #     logging.info("KEYUP A")
        #     pyautogui.keyUp(self.mapping.button['B'])

        # self.incoming = self.ser.read()
        self.ser.write(b'C')


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['B'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['B'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
