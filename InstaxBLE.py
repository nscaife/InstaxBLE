#!/usr/bin/env python3

from math import ceil
from struct import pack, unpack_from
from time import sleep
import asyncio

# Try to import Types with a relative import first
try:
    from .Types import EventType, InfoType, PrinterSettings
    from . import LedPatterns
except ImportError:
    # If that fails (which it will if this file is being run directly),
    # try an absolute import instead
    from Types import EventType, InfoType, PrinterSettings
    import LedPatterns

import argparse

from bleak import BleakScanner, BleakClient
import sys
from PIL import Image
from io import BytesIO


class InstaxBLE:
    def __init__(
        self,
        device_address=None,
        device_name=None,
        print_enabled=False,
        dummy_printer=False,
        verbose=False,
        quiet=False,
        image_path=None):
        """
        Initialize the InstaxBLE class.
        deviceAddress: if specified, will only connect to a printer with this address.
        printEnabled: by default, actual printing is disabled to prevent misprints.
        """
        # BLE
        self.serviceUUID = '70954782-2d83-473d-9e5f-81e1d02d5273'
        self.writeCharUUID = '70954783-2d83-473d-9e5f-81e1d02d5273'
        self.notifyCharUUID = '70954784-2d83-473d-9e5f-81e1d02d5273'
        self.client = None  # BleakClient replaces peripheral

        self.quiet = quiet
        self.dummyPrinter = dummy_printer
        self.printerSettings = PrinterSettings['mini'] if self.dummyPrinter else None
        self.chunkSize = PrinterSettings['mini']['chunkSize'] if self.dummyPrinter else 0
        self.printEnabled = print_enabled
        self.deviceName = device_name.upper() if device_name else None
        self.deviceAddress = device_address.upper() if device_address else None
        self.image_path = image_path
        self.verbose = verbose if not self.quiet else False
        self.packetsForPrinting = []
        self.pos = (0, 0, 0, 0)
        self.batteryState = 0
        self.batteryPercentage = 0
        self.photosLeft = 0
        self.isCharging = False
        self.imageSize = (PrinterSettings['mini']['width'], PrinterSettings['mini']['height']) if self.dummyPrinter else (0, 0)
        self.waitingForResponse = False
        self.cancelled = False
        self.response_event = asyncio.Event()  # For async waiting

    def log(self, msg):
        """ Print a debug message"""
        if self.verbose:
            print(msg)

    def display_current_status(self):
        """ Display an overview of the current printer state """
        print("\nPrinter details: ")
        # print(f"Device name:         {self.printerSettings['modelName']}")
        if self.printerSettings:
            print(f"Model:               {self.printerSettings['modelName']}")
            print(f"Photos left:         {self.photosLeft}/10")
            print(f"Battery level:       {self.batteryPercentage}%")
            print(f"Charging:            {self.isCharging}")
            print(f"Required image size: {self.printerSettings['width']}x{self.printerSettings['height']}px")
        else:
            print("Printer settings not yet determined")
        print("")

    def parse_printer_response(self, event, packet):
        """ Parse the response packet and print the result """
        self.log(f"Parsing response for event: {event}")
        self.waitingForResponse = False

        if event == EventType.XYZ_AXIS_INFO:
            x, y, z, o = unpack_from('<hhhB', packet[6:-1])
            self.pos = (x, y, z, o)
        elif event == EventType.LED_PATTERN_SETTINGS:
            pass
        elif event == EventType.SUPPORT_FUNCTION_INFO:
            # Debug: check both byte 6 and 7 to see which has the info type
            self.log(f'Payload bytes: packet[6]={packet[6]:02x}, packet[7]={packet[7]:02x}')
            try:
                infoType = InfoType(packet[7])
            except ValueError:
                self.log(f'Unknown InfoType at packet[7]: {packet[7]}')
                # Try packet[6] instead
                try:
                    infoType = InfoType(packet[6])
                    self.log(f'Found InfoType at packet[6]: {infoType}')
                except ValueError:
                    self.log(f'Unknown InfoType at packet[6]: {packet[6]}')
                    return

            if infoType == InfoType.IMAGE_SUPPORT_INFO:
                w, h = unpack_from('>HH', packet[8:12])
                # self.log(self.prettify_bytearray(packet[8:12]))
                # self.log(f'image size: {w}x{h}')
                self.imageSize = (w, h)
                if (w, h) == (600, 800):
                    self.printerSettings = PrinterSettings['mini']
                elif (w, h) == (800, 800):
                    self.printerSettings = PrinterSettings['square']
                elif (w, h) == (1260, 840):
                    self.printerSettings = PrinterSettings['wide']
                else:
                    exit(f'Unknown image size from printer: {w}x{h}')

                self.chunkSize = self.printerSettings['chunkSize']

            elif infoType == InfoType.BATTERY_INFO:
                self.batteryState, self.batteryPercentage = unpack_from('>BB', packet[8:10])
                # self.log(f'battery state: {self.batteryState}, battery percentage: {self.batteryPercentage}')
            elif infoType == InfoType.PRINTER_FUNCTION_INFO:
                dataByte = packet[8]
                self.photosLeft = dataByte & 15
                self.isCharging = ((1 << 7) & dataByte) != 0  # Fix operator precedence
                # self.log(f'photos left: {self.photosLeft}')
                # if self.isCharging:
                #     self.log('Printer is charging')
                # else:
                #     self.log('Printer is running on battery')

        elif event == EventType.PRINT_IMAGE_DOWNLOAD_START:
            asyncio.create_task(self.handle_image_packet_queue())

        elif event == EventType.PRINT_IMAGE_DOWNLOAD_DATA:
            asyncio.create_task(self.handle_image_packet_queue())

        elif event == EventType.PRINT_IMAGE_DOWNLOAD_END:
            asyncio.create_task(self.handle_image_packet_queue())

        elif event == EventType.PRINT_IMAGE_DOWNLOAD_CANCEL:
            pass

        elif event == EventType.PRINT_IMAGE:
            self.log('received print confirmation')
            pass

        else:
            self.log(f'Uncaught response from printer. Eventype: {event}')

    async def handle_image_packet_queue(self):
        if len(self.packetsForPrinting) > 0 and not self.cancelled:
            if len(self.packetsForPrinting) % 10 == 0:
                self.log(f"Img packets left to send: {len(self.packetsForPrinting)}")
            packet = self.packetsForPrinting.pop(0)
            await self.send_packet(packet)

    def notification_handler(self, sender, data):
        """ Gets called whenever the printer replies and handles parsing the received data
        In bleak, this receives (sender, data) where data is bytes"""
        packet = bytearray(data)  # Convert bytes to bytearray
        self.log('Notification handler called')
        self.log(f'\tPacket size: {len(packet)}')
        self.log(f'\tPacket (first 40 bytes): {self.prettify_bytearray(packet[:40])}')

        if len(packet) < 8:
            self.log(f"\tError: response packet size should be >= 8 (was {len(packet)})!")
            self.waitingForResponse = False  # Critical: unblock waiting threads
            return
        elif not self.validate_checksum(packet):
            self.log("\tResponse packet checksum was invalid!")
            self.waitingForResponse = False  # Critical: unblock waiting threads
            return

        header, length, op1, op2 = unpack_from('>HHBB', packet)

        try:
            event = EventType((op1, op2))
        except ValueError:
            self.log(f"Unknown EventType: ({op1}, {op2})")
            self.waitingForResponse = False
            return

        self.parse_printer_response(event, packet)

    async def connect(self, timeout=0):
        """ Connect to the printer. Stops trying after <timeout> seconds. """
        if self.dummyPrinter:
            return

        device = await self.find_device(timeout=timeout)
        if device:
            try:
                self.log(f"Connecting to {device.name} [{device.address}]")
                self.client = BleakClient(device)
                await self.client.connect()
                self.log(f"Connected")

                # Set up notification handler
                self.log('Attaching notification_handler')
                await self.client.start_notify(self.notifyCharUUID, self.notification_handler)
                self.log('Notification handler attached successfully')

                # Give BLE stack time to set up notifications
                await asyncio.sleep(0.5)

                await self.get_printer_info()
                self.log('Waiting for printer responses...')
                await asyncio.sleep(2)  # Give time for all responses to come back
                self.log('Done waiting, displaying status...')
                self.display_current_status()

            except Exception as e:
                if not self.quiet:
                    self.log(f'Error connecting: {e}')
                raise

    async def disconnect(self):
        """ Disconnect from the printer (if connected) """
        if self.dummyPrinter:
            return
        if self.client:
            if self.client.is_connected:
                self.log('Disconnecting...')
                try:
                    # Stop notifications before disconnecting
                    await self.client.stop_notify(self.notifyCharUUID)
                    self.log('Stopped notifications')
                except Exception as e:
                    self.log(f'Error stopping notifications: {e}')

                await self.client.disconnect()
                self.log("Disconnected")

    def cancel_print(self):
        self.packetsForPrinting = []
        self.waitingForResponse = False
        self.send_packet(self.create_packet(EventType.PRINT_IMAGE_DOWNLOAD_CANCEL))

    def enable_printing(self):
        """ Enable printing. """
        self.printEnabled = True

    def disable_printing(self):
        """ Disable printing. """
        self.printEnabled = False

    async def find_device(self, timeout=0):
        """" Scan for our device and return it when found """
        self.log('Searching for instax printer...')

        def match_device(device, adv_data):
            """Check if this device matches our search criteria"""
            name = device.name or ""
            address = device.address or ""

            if (self.deviceName and name.startswith(self.deviceName)) or \
               (self.deviceAddress and address.upper() == self.deviceAddress) or \
               (self.deviceName is None and self.deviceAddress is None and
                name.startswith('INSTAX-') and name.endswith('(IOS)')):
                return True
            return False

        try:
            if timeout > 0:
                # Scan for specified timeout
                self.log(f"Scanning for {timeout} seconds...")
                device = await BleakScanner.find_device_by_filter(match_device, timeout=timeout)
            else:
                # Scan until found
                while True:
                    self.log("Scanning...")
                    device = await BleakScanner.find_device_by_filter(match_device, timeout=5.0)
                    if device:
                        break

            if device:
                self.log(f"Found printer: {device.name} [{device.address}]")
                return device
            else:
                self.log("No matching printer found")
                return None

        except KeyboardInterrupt:
            self.log("Scan cancelled by user")
            sys.exit()

    def create_color_payload(self, colorArray, speed, repeat, when):
        """
        Create a payload for a color pattern. See send_led_pattern for details.
        """
        payload = pack('BBBB', when, len(colorArray), speed, repeat)
        for color in colorArray:
            payload += pack('BBB', color[0], color[1], color[2])
        return payload

    async def send_led_pattern(self, pattern, speed=5, repeat=255, when=0):
        """ Send a LED pattern to the Instax printer.
            colorArray: array of BGR(!) values to use in animation, e.g. [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
            speed: time per frame/color: higher is slower animation
            repeat: 0 = don't repeat (so play once), 1-254 = times to repeat, 255 = repeat forever
            when: 0 = normal, 1 = on print, 2 = on print completion, 3 = pattern switch """
        payload = self.create_color_payload(pattern, speed, repeat, when)
        packet = self.create_packet(EventType.LED_PATTERN_SETTINGS, payload)
        await self.send_packet(packet)

    def prettify_bytearray(self, value):
        """ Helper funtion to convert a bytearray to a string of hex values. """
        return ' '.join([f'{x:02x}' for x in value])

    def create_checksum(self, bytearray):
        """ Create a checksum for a given packet. """
        return (255 - (sum(bytearray) & 255)) & 255

    def create_packet(self, eventType, payload=b''):
        """ Create a packet to send to the printer. """
        if isinstance(eventType, EventType):  # allows passing in an event or a value directly
            eventType = eventType.value

        header = b'\x41\x62'  # 'Ab' means client to printer, 'aB' means printer to client
        opCode = bytes([eventType[0], eventType[1]])
        packetSize = pack('>H', 7 + len(payload))
        packet = header + packetSize + opCode + payload
        packet += pack('B', self.create_checksum(packet))
        return packet

    def validate_checksum(self, packet):
        """ Validate the checksum of a packet. """
        return (sum(packet) & 255) == 255

    async def send_packet(self, packet):
        """ Send a packet to the printer """
        if not self.dummyPrinter:
            if not self.client:
                self.log("ERROR: no client to send packet to")
                return
            elif not self.client.is_connected:
                self.log("ERROR: client not connected")
                return

        try:
            if self.waitingForResponse:
                self.log("Waiting for previous response...")

            # Add timeout to prevent infinite hangs
            timeout_counter = 0
            max_timeout = 5.0  # 5 seconds timeout
            while self.waitingForResponse and not self.dummyPrinter and not self.cancelled:
                await asyncio.sleep(0.05)
                timeout_counter += 0.05
                if timeout_counter >= max_timeout:
                    self.log(f"WARNING: Timeout waiting for response after {max_timeout}s, continuing anyway")
                    self.waitingForResponse = False
                    break

            header, length, op1, op2 = unpack_from('>HHBB', packet)
            try:
                event = EventType((op1, op2))
            except Exception:
                event = 'Unknown event'

            self.log(f'Sending packet: {event}')
            self.log(f'Packet data (first 40 bytes): {self.prettify_bytearray(packet[:40])}')

            self.waitingForResponse = True
            smallPacketSize = 182
            numberOfParts = ceil(len(packet) / smallPacketSize)
            self.log(f"Packet will be sent in {numberOfParts} part(s)")
            for subPartIndex in range(numberOfParts):
                subPacket = packet[subPartIndex * smallPacketSize:subPartIndex * smallPacketSize + smallPacketSize]

                if not self.dummyPrinter:
                    try:
                        # Use write_gatt_char with response=False (equivalent to write_command)
                        await self.client.write_gatt_char(self.writeCharUUID, subPacket, response=False)
                        self.log(f"Sent part {subPartIndex + 1}/{numberOfParts}")
                    except Exception as e:
                        self.log(f"ERROR writing packet: {e}")
                        self.waitingForResponse = False
                        raise

        except KeyboardInterrupt:
            self.cancelled = True
            await self.disconnect()
            sys.exit('Cancelled')
        except Exception as e:
            self.log(f"CRITICAL ERROR in send_packet: {type(e).__name__}: {e}")
            self.waitingForResponse = False
            raise

    async def print_image(self, imgSrc):
        """
        print an image. Either pass a path to an image (as a string) or pass
        the bytearray to print directly
        """
        self.log(f'printing image "{imgSrc}"')
        if self.photosLeft == 0 and not self.dummyPrinter:
            self.log("Can't print: no photos left")
            return

        imgData = imgSrc
        if isinstance(imgSrc, str):  # if it's a path, load the image contents
            image = Image.open(imgSrc)
            imgData = self.pil_image_to_bytes(image, max_size_kb=105)
        elif isinstance(imgSrc, BytesIO):
            imgSrc.seek(0)  # Go to the start of the BytesIO object
            image = Image.open(imgSrc)
            imgData = self.pil_image_to_bytes(image, max_size_kb=105)

        # self.log(f"len of imagedata: {len(imgData)}")
        self.packetsForPrinting = [
            # \x02\x00\x00\x00 payload made of four bytes: pictureType, picturePrintOption, picturePrintOption2, zero
            self.create_packet(EventType.PRINT_IMAGE_DOWNLOAD_START, b'\x02\x00\x00\x00' + pack('>I', len(imgData)))
        ]

        # divide image data up into chunks of <chunkSize> bytes and pad the last chunk with zeroes if needed
        imgDataChunks = [imgData[i:i + self.chunkSize] for i in range(0, len(imgData), self.chunkSize)]
        if len(imgDataChunks[-1]) < self.chunkSize:
            imgDataChunks[-1] = imgDataChunks[-1] + bytes(self.chunkSize - len(imgDataChunks[-1]))

        # create a packet from each of our chunks, this includes adding the chunk number
        for index, chunk in enumerate(imgDataChunks):
            imgDataChunks[index] = pack('>I', index) + chunk  # add chunk number as int (4 bytes)
            self.packetsForPrinting.append(self.create_packet(EventType.PRINT_IMAGE_DOWNLOAD_DATA, imgDataChunks[index]))

        self.packetsForPrinting.append(self.create_packet(EventType.PRINT_IMAGE_DOWNLOAD_END))

        if self.printEnabled:
            self.packetsForPrinting.append(self.create_packet(EventType.PRINT_IMAGE))
            self.packetsForPrinting.append(self.create_packet((0, 2), b'\x02'))
        elif not self.quiet:
            self.log("Printing is disabled, sending all packets except the actual print command")

        # send the first packet from our list, the packet handler will take care of the rest
        if not self.dummyPrinter:
            packet = self.packetsForPrinting.pop(0)
            await self.send_packet(packet)

    def print_services(self):
        """ Get and display and overview of the printer's services and characteristics """
        self.log("Successfully connected, listing services...")
        services = self.peripheral.services()
        service_characteristic_pair = []
        for service in services:
            for characteristic in service.characteristics():
                service_characteristic_pair.append((service.uuid(), characteristic.uuid()))

        for i, (service_uuid, characteristic) in enumerate(service_characteristic_pair):
            self.log(f"{i}: {service_uuid} {characteristic}")

    async def get_printer_orientation(self):
        """ Get the current XYZ orientation of the printer """
        packet = self.create_packet(EventType.XYZ_AXIS_INFO)
        await self.send_packet(packet)

    async def get_printer_status(self):
        """ Get the printer's status"""
        packet = self.create_packet(EventType.SUPPORT_FUNCTION_INFO, pack('>B', InfoType.PRINTER_FUNCTION_INFO.value))
        await self.send_packet(packet)

    async def get_printer_info(self):
        """ Get and display the printer's status and info, like photos left and battery level """
        self.log("Getting printer info...")

        self.log("Requesting IMAGE_SUPPORT_INFO")
        packet = self.create_packet(EventType.SUPPORT_FUNCTION_INFO, pack('>B', InfoType.IMAGE_SUPPORT_INFO.value))
        await self.send_packet(packet)
        await asyncio.sleep(0.5)  # Give printer time to respond

        self.log("Requesting BATTERY_INFO")
        packet = self.create_packet(EventType.SUPPORT_FUNCTION_INFO, pack('>B', InfoType.BATTERY_INFO.value))
        await self.send_packet(packet)
        await asyncio.sleep(0.5)  # Give printer time to respond

        self.log("Requesting PRINTER_FUNCTION_INFO")
        await self.get_printer_status()
        await asyncio.sleep(0.5)  # Give printer time to respond

    def pil_image_to_bytes(self, img: Image.Image, max_size_kb: int = None) -> bytearray:
        """ Convert a PIL image to a bytearray """
        img_buffer = BytesIO()

        # Convert the image to RGB mode if it's in RGBA mode
        if img.mode == 'RGBA':
            img = img.convert('RGB')

        # Resize the image to <imageSize> pixels
        img = img.resize(self.imageSize, Image.Resampling.LANCZOS)

        def save_img_with_quality(quality):
            img_buffer.seek(0)
            img.save(img_buffer, format='JPEG', quality=quality)
            return img_buffer.tell() / 1024

        if max_size_kb is not None:
            low_quality, high_quality = 1, 100
            current_quality = 75
            closest_quality = current_quality
            min_target_size_kb = max_size_kb * 0.9

            while low_quality <= high_quality:
                output_size_kb = save_img_with_quality(current_quality)
                # self.log(f"current output quality: {current_quality}, current size: {output_size_kb}")

                if output_size_kb <= max_size_kb and output_size_kb >= min_target_size_kb:
                    closest_quality = current_quality
                    break

                if output_size_kb > max_size_kb:
                    high_quality = current_quality - 1
                else:
                    low_quality = current_quality + 1

                current_quality = (low_quality + high_quality) // 2
                closest_quality = current_quality

            # Save the image with the closest_quality
            save_img_with_quality(closest_quality)
            self.log(f'Saved img with quality of {closest_quality}')
        else:
            img.save(img_buffer, format='JPEG')

        return bytearray(img_buffer.getvalue())

    async def wait_one_minute(self):
        """ Wait for one minute. Hacky way of preventing disconnecting too soon """
        if not self.quiet:
            print("Waiting for one minute...")
        await asyncio.sleep(60)


async def main(args={}):
    """ Example usage of the InstaxBLE class """
    instax = InstaxBLE(**args)
    try:
        # To prevent misprints during development this script sends all the
        # image data except the final 'go print' command. To enable printing
        # uncomment the next line, or pass --print-enabled when calling
        # this script

        # instax.enable_printing()
        await instax.connect()
        # Set a rainbow effect to be shown while printing and a pulsating
        # green effect when printing is done
        await instax.send_led_pattern(LedPatterns.rainbow, when=1)
        await instax.send_led_pattern(LedPatterns.pulseGreen, when=2)
        # you can also read the current accelerometer values if you want
        # while True:
        #     await instax.get_printer_orientation()
        #     await asyncio.sleep(.5)

        # send your image (.jpg) to the printer by
        # passing the image_path as an argument when calling
        # this script, or by specifying the path in your code
        if instax.image_path:
            await instax.print_image(instax.image_path)
        elif instax.printerSettings:
            await instax.print_image(instax.printerSettings['exampleImage'])
        else:
            print("Cannot print: printer settings not determined (image size unknown)")
            return
        await instax.wait_one_minute()

    except Exception as e:
        print(type(e).__name__, __file__, e.__traceback__.tb_lineno)
        instax.log(f'Error: {e}')
    finally:
        print('finally, disconnect')
        await instax.disconnect()  # all done, disconnect


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--device-address')
    parser.add_argument('-n', '--device-name')
    parser.add_argument('-p', '--print-enabled', action='store_true')
    parser.add_argument('-d', '--dummy-printer', action='store_true')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-q', '--quiet', action='store_true')
    parser.add_argument('-i', '--image-path', help='Path to the image file')
    args = parser.parse_args()

    asyncio.run(main(vars(args)))
