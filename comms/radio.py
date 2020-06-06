"""XBee radio communications class
Setup + Troubleshooting:

802.15.4 devices
    Click Load default firmware settings in the Radio Configuration toolbar to
    load the default values for the device firmware. Make sure API mode
    (API1 or API2) is enabled. To do so, set the AP parameter value to 1
    (API mode without escapes) or 2 (API mode with escapes).
    Configure ID (PAN ID) setting to CAFE.
    Configure CH (Channel setting) to C.
    Click Write radio settings in the Radio Configuration toolbar to apply the
    new values to the module.
    Once you have configured both modules, check to make sure they can see
    each other. Click Discover radio modules in the same network, the second
    button of the device panel in the Radio Modules view. The other device
    must be listed in the Discovering remote devices dialog.

Troubleshooting:
    If you are getting the error
    Could not open port "/dev/USBXXX", permission denied
    then make sure that your user has usb port access without sudo
    you can do this by adding user into dialout group. Google this.

"""
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import XBeeException
import time

RADIO_PORT_1 = "/dev/ttyUSB0"
RADIO_PORT_2 = "TODO: doesn't exist yet"
BAUD_RATE = 9600


class Radio(object):
    # current xbee only can send once every ~60ms, sending faster may block
    MESSAGE_DELAY = .1

    def __init__(self, is_second_radio=False):
        # Find our XBee device connected to this computer
        port = RADIO_PORT_2 if is_second_radio else RADIO_PORT_1
        self.device = XBeeDevice(port, BAUD_RATE)

        # TODO: sometimes it errors about operating mode, try replugging xbee
        self.device.open()

        # Obtain the remote XBee devices from the XBee network.
        xbee_network = self.device.get_network()

        # Try to find devices
        xbee_network.start_discovery_process()
        time.sleep(3)  # wait a few seconds to find all of the xbees
        xbee_network.stop_discovery_process()
        self.net_devs = xbee_network.get_devices()
        if not self.net_devs:
            raise RuntimeError("Cound not find any XBEE devices on network")

    def send(self, message):
        for remote_device in self.net_devs:
            try:
                start = time.time()
                # asynchronous send is fast for first msg, but waits if more
                # long messages (>30?) take longer because they must be split
                try:
                    self.device.send_data_async(remote_device, message)
                except Exception as e:
                    print('xbee error - something using same port? (xtcu):')
                    print(e)
                    # TODO: reconnect when error?
                delta = time.time() - start
                if delta > .003:
                    print('xbee send is taking a long time, too long/many?')
                    print("time taken: " + str(delta))
                    print("message length: " + str(len(message)))
            except XBeeException as xbee_exp:
                print(str(xbee_exp))

    def read(self):
        for remote_device in self.net_devs:
            try:
                return self.device.read_data()
            except XBeeException as xbee_exp:
                print(str(xbee_exp))

    def close(self):
        if self.device.is_open():
            self.device.close()
