# ble_uart_bridge.py
# BLE command receiver -> UART actions for the Adafruit QT Py ESP32-C3.
#
# The ESP32-C3 has BLE (no BT Classic), so we expose a BLE peripheral that
# implements the Nordic UART Service (NUS). Any generic BLE terminal app
# (nRF Connect, LightBlue, Adafruit Bluefruit Connect) can connect and send
# lines of text. Each line is parsed as a command and dispatched to a handler
# that talks to the hardware UART.
#
# Only the built-in `bluetooth` and `machine` modules are used -- no extra
# libraries to install.

import bluetooth
import struct
import time
from machine import Pin, UART
from micropython import const

# ---------------------------------------------------------------------------
# BLE IRQ event constants (from the MicroPython bluetooth docs)
# ---------------------------------------------------------------------------
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_MTU_EXCHANGED = const(21)

# Advertising payload flags
_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)

# Nordic UART Service (NUS) UUIDs -- the de-facto standard, recognised by apps.
_UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
# RX = central writes to us (commands come IN here)
_UART_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
# TX = we notify the central (replies go OUT here)
_UART_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

_FLAG_WRITE = const(0x0008)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_NOTIFY = const(0x0010)

_UART_RX = (_UART_RX_UUID, _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE)
_UART_TX = (_UART_TX_UUID, _FLAG_NOTIFY)
_UART_SERVICE = (_UART_SERVICE_UUID, (_UART_TX, _UART_RX))


def _advertising_payload(name=None, services=None):
    """Build a BLE advertising payload (name + 128-bit service UUID)."""
    payload = bytearray()

    def _append(adv_type, value):
        payload.extend(struct.pack("BB", len(value) + 1, adv_type) + value)

    # General discoverable, BR/EDR not supported (we're BLE-only).
    _append(_ADV_TYPE_FLAGS, struct.pack("B", 0x02 | 0x04))
    if name:
        _append(_ADV_TYPE_NAME, name.encode())
    if services:
        for uuid in services:
            _append(_ADV_TYPE_UUID128_COMPLETE, bytes(uuid))
    return payload


class BLECommandServer:
    """Minimal NUS peripheral: receives lines, calls a dispatcher, can reply."""

    def __init__(self, name="QTPY-C3", on_line=None):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.config(mtu=247)        # offer a bigger MTU; result = min(ours, phone's)
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services(
            (_UART_SERVICE,)
        )
        self._connections = set()
        self._on_line = on_line          # callback(str) -> optional reply str
        self._rx_buffer = b""            # accumulate until newline
        self._pending = []               # complete lines waiting for poll()
        self._chunk = 20                 # payload bytes per notification (MTU - 3)
        self._payload = _advertising_payload(
            name=name, services=[_UART_SERVICE_UUID]
        )
        self._advertise()

    def _advertise(self, interval_us=250000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
        print("BLE: advertising, waiting for a connection...")

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("BLE: central connected", conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            print("BLE: central disconnected", conn_handle)
            self._advertise()  # re-advertise so we can be found again
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                self._drain_lines()
        elif event == _IRQ_MTU_EXCHANGED:
            conn_handle, mtu = data
            self._chunk = mtu - 3        # 3 bytes of ATT header per notification

    def _drain_lines(self):
        # Commands are newline-delimited. A single BLE write may contain
        # several lines, or a partial line -- buffer handles both.
        while b"\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
            text = line.decode().strip()
            if text:
                self._pending.append(text)  # run by poll(), not inside the IRQ

    def poll(self):
        """Run queued command lines and send their replies. Call from the main loop."""
        while self._pending:
            text = self._pending.pop(0)
            reply = None
            if self._on_line:
                try:
                    reply = self._on_line(text)
                except Exception as e:  # keep the link alive on handler errors
                    reply = "ERR {}".format(e)
            if reply is not None:
                self.send(str(reply))

    def send(self, text):
        """Notify the connected central, split into chunks that fit one notification."""
        data = (text + "\n").encode()
        for conn in tuple(self._connections):  # the IRQ may change the set meanwhile
            for i in range(0, len(data), self._chunk):
                if conn not in self._connections:
                    break  # the phone left mid-reply
                self._notify(conn, data[i:i + self._chunk])

    def _notify(self, conn, chunk):
        for _ in range(20):
            try:
                self._ble.gatts_notify(conn, self._tx_handle, chunk)
                return
            except OSError:  # send queue full (or the phone just left)
                if conn not in self._connections:
                    return
                time.sleep_ms(10)

    def is_connected(self):
        return bool(self._connections)


# ---------------------------------------------------------------------------
# UART side: the actions commands trigger
# ---------------------------------------------------------------------------
# QT Py ESP32-C3: GPIO21 = TX pad, GPIO20 = RX pad (UART0 is the USB REPL,
# so we use UART1 on these pads). Adjust pins/baud to your target devices.
uart = UART(1, baudrate=115200, tx=Pin(21), rx=Pin(20), timeout=1000, rxbuf=1024)

# M7E opcodes and settings. MicroPython swaps _CONST names for their values
# at compile time, so they must be defined above the code that uses them.
# these are all taken from source code somewhere else--not found experimentally. 
_OP_BOOT_FIRMWARE = const(0x04)
_OP_GET_PROGRAM = const(0x0C)
_OP_READ_TAG_MULTIPLE = const(0x22)   # timed inventory into the tag buffer
_OP_GET_TAG_BUFFER = const(0x29)
_OP_CLEAR_TAG_BUFFER = const(0x2A)
_OP_SET_ANTENNA_PORT = const(0x91)
_OP_SET_READ_POWER = const(0x92)
_OP_SET_TAG_PROTOCOL = const(0x93)
_OP_SET_WRITE_POWER = const(0x94)
_OP_SET_REGION = const(0x97)

_STATUS_NO_TAGS = const(0x0400)
_META_RSSI = const(0x0002)            # metadata flag: attach RSSI to each tag
_INVENTORY_MS = const(500)            # radio time per inventory

_REGION = const(0x01)                 # North America, 902-928 MHz: use your country's code
_READ_POWER = const(1000)             # centi-dBm: 10.00 dBm
_WRITE_POWER = const(1500)            # centi-dBm: 15.00 dBm 

_tags_left = 0                        # set by ITAG, counted down by POP

# Optional: NeoPixel for visual feedback (GPIO2 on this board).
from neopixel import NeoPixel
_pixel = NeoPixel(Pin(2, Pin.OUT), 1)


def _set_pixel(r, g, b):
    _pixel[0] = (r, g, b)
    _pixel.write()


# --- Command handlers -------------------------------------------------------
# Each handler takes the list of arguments (command word already stripped)
# and returns a string reply (or None for no reply).

def cmd_ping(args):
    return "pong"


# SEND, HEX and READ bypass the M7E framing: handy for experiments, but READ
# can swallow a reply that another command is waiting for.
def cmd_send(args):
    # SEND <text...>  -> write raw text to the UART
    payload = " ".join(args)
    uart.write(payload.encode())
    return "sent {} bytes".format(len(payload))


def cmd_hex(args):
    # HEX <aa bb cc> -> write raw bytes to the UART
    try:
        data = bytes(int(b, 16) for b in args)
    except ValueError:
        return "ERR bad hex"
    uart.write(data)
    return "sent {} bytes".format(len(data))


def cmd_read(args):
    # READ -> drain whatever the UART has buffered and return it
    data = uart.read()
    if not data:
        return "(nothing)"
    return data.hex()


def cmd_led(args):
    # LED <r> <g> <b>  (0-255 each)
    try:
        r, g, b = (int(x) & 0xFF for x in args[:3])
    except (ValueError, TypeError):
        return "ERR usage: LED r g b"
    _set_pixel(r, g, b)
    return "ok"

def cmd_itag(args):
    """ITAG -> inventory every tag in range and reply with how many were found."""
    global _tags_left
    status, _ = transact(_OP_CLEAR_TAG_BUFFER)  # forget the tags from the last run
    if status:
        return "ERR clear buffer, status {:04X}".format(status)
    # 00 = no filter, 0010 = report the count in 4 bytes, then the radio time in ms.
    # imagine using little endian in the year of our lord 2026. Couldn't be me.
    payload = struct.pack(">BHH", 0x00, 0x0010, _INVENTORY_MS)
    # The reply only comes once the radio time is over, so wait longer than that.
    status, data = transact(_OP_READ_TAG_MULTIPLE, payload, _INVENTORY_MS + 1000)
    if status == _STATUS_NO_TAGS:
        _tags_left = 0
        return "Found 0 tags"
    if status:
        return "ERR inventory, status {:04X}".format(status)
    # data = option (1), search flags (2), tag count (1 or 4 bytes, always last)
    _tags_left = data[-1] if len(data) in (4, 5) else struct.unpack(">I", data[-4:])[0]
    return "Found {} tags".format(_tags_left)


def cmd_pop(args):
    """POP -> fetch the tags found by the last ITAG, one 'EPC RSSI' line each."""
    global _tags_left
    if not _tags_left:
        return "No tags waiting; run ITAG first"
    lines = []
    count = 0
    while _tags_left > 0:  # one frame holds about a dozen tags, so ask until done
        # Ask for RSSI only (the EPC always comes); 00 = no read options.
        status, data = transact(_OP_GET_TAG_BUFFER, struct.pack(">HB", _META_RSSI, 0x00))
        if status:
            lines.append("ERR tag buffer, status {:04X}".format(status))
            break
        tags = _parse_tag_buffer(data)
        if not tags:
            break
        for epc, rssi in tags:
            lines.append("{} {}".format(epc.hex(), rssi))
        count += len(tags)
        _tags_left -= len(tags)
    _tags_left = 0
    return "\n".join(["{} tags".format(count)] + lines)


def cmd_setup(args):
    # SETUP -> run the M7E bring-up again, e.g. if it was powered after the C3
    m7e_setup()
    return "ok"

# --- M7E protocol tools ----------------------------------------------------
# These are specific to the M7E over UART. A frame is bytes: FF header, LEN,
# opcode, payload (or status + data in a reply), then a 2-byte CRC.
# we need to parse both ways: phone sends order, we build frame, send to M7E; M7E sends response, we parse frame, send to phone.

def crc16(data):
    # ThingMagic CRC: polynomial 0x1021, start 0xFFFF, data shifted in
    # 4 bits at a time. (The C code's 16-entry table is just 0x1021 * i.)
    crc = 0xFFFF
    for b in data:
        for nibble in (b >> 4, b & 0x0F):
            crc = (((crc << 4) | nibble) ^ (0x1021 * (crc >> 12))) & 0xFFFF
    return crc

def build_frame(opcode, payload=b""):
    body = bytes((len(payload), opcode)) + payload
    return b"\xff" + body + struct.pack(">H", crc16(body))


def _read_exact(n, start, timeout_ms):
    """Read exactly n bytes from the UART, or raise once timeout_ms has passed."""
    buf = b""
    while len(buf) < n:
        chunk = uart.read(n - len(buf))
        if chunk:
            buf += chunk
        elif time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
            raise OSError("M7E did not answer in time")
    return buf


def transact(opcode, payload=b"", timeout_ms=1000):
    """Send one command frame and return (status, data) from the M7E's reply.

    data has the header, LEN, opcode, status and CRC already removed.
    timeout_ms must be longer than any radio time inside the payload.
    Raises OSError on a timeout or a corrupted reply.
    """
    while uart.any():  # drop stale bytes so they can't pose as our reply
        uart.read()
    uart.write(build_frame(opcode, payload))
    start = time.ticks_ms()
    while True:
        while _read_exact(1, start, timeout_ms) != b"\xff":
            pass  # skip noise until a start byte
        head = _read_exact(4, start, timeout_ms)  # LEN, OPCODE, STATUS x2
        rest = _read_exact(head[0] + 2, start, timeout_ms)  # DATA + CRC
        if crc16(head + rest[:-2]) != struct.unpack(">H", rest[-2:])[0]:
            raise OSError("bad CRC in M7E reply")
        if head[1] == opcode:
            return (head[2] << 8) | head[3], rest[:-2]
        # A late reply to an earlier command: ignore it and read the next frame.


def _parse_tag_buffer(data):
    """Split the data of a GET_TAG_BUFFER reply into (epc, rssi) pairs."""
    # data = metadata flags (2), read options (1), tags in this frame (1), records
    flags = (data[0] << 8) | data[1]
    if flags != _META_RSSI:
        raise ValueError("unexpected metadata flags {:04X}".format(flags))
    tags = []
    i = 4
    for _ in range(data[3]):
        # Record: RSSI (1), tag ID length in bits (2), PC (2), EPC, tag CRC (2)
        rssi = data[i] - 256 if data[i] > 127 else data[i]  # signed dBm
        id_len = (((data[i + 1] << 8) | data[i + 2]) + 7) // 8  # PC + EPC + CRC
        pc = (data[i + 3] << 8) | data[i + 4]
        start = i + 5 + (2 if pc & 0x0200 else 0)  # XI bit: an XPC word follows the PC
        epc = data[start:start + (pc >> 11) * 2]  # top 5 bits of PC = EPC length in words
        tags.append((epc, rssi))
        i += 3 + id_len
    return tags


def m7e_setup():
    """Bring the M7E from power-on to ready for Gen2 inventories."""
    status, data = transact(_OP_GET_PROGRAM)
    if not status and data[0] & 0x03 == 1:  # 1 = still in the bootloader
        transact(_OP_BOOT_FIRMWARE, timeout_ms=3000)
    for opcode, payload in (
        (_OP_SET_TAG_PROTOCOL, b"\x00\x05"),  # 0x0005 = Gen2
        (_OP_SET_ANTENNA_PORT, b"\x01\x01"),  # TX port 1, RX port 1
        (_OP_SET_REGION, bytes((_REGION,))),
        (_OP_SET_READ_POWER, struct.pack(">h", _READ_POWER)),
        (_OP_SET_WRITE_POWER, struct.pack(">h", _WRITE_POWER)),
    ):
        status, _ = transact(opcode, payload)
        if status:
            raise OSError("M7E setup: op {:02X} failed, status {:04X}".format(opcode, status))

# The dispatch table: command word -> handler. Add new commands here.
COMMANDS = {
    "PING": cmd_ping,
    "SEND": cmd_send,
    "HEX": cmd_hex,
    "READ": cmd_read,
    "LED": cmd_led,
    "SETUP": cmd_setup,
    "ITAG": cmd_itag,
    "POP": cmd_pop,
}


def dispatch(line):
    """Parse one command line and run its handler. Returns a reply string."""
    parts = line.split()
    word = parts[0].upper()
    args = parts[1:]
    handler = COMMANDS.get(word)
    if handler is None:
        return "ERR unknown '{}' (try: {})".format(word, " ".join(COMMANDS))
    return handler(args)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    try:
        m7e_setup()
        print("M7E: ready")
    except Exception as e:  # e.g. module unpowered; retry later with SETUP
        print("M7E: setup failed:", e)
    server = BLECommandServer(name="QTPY-C3", on_line=dispatch)
    _set_pixel(0, 0, 8)  # dim blue = idle/advertising
    connected = False
    while True:
        server.poll()  # run queued commands here, outside the BLE IRQ
        # Visual heartbeat so you know which state you're in.
        if server.is_connected() and not connected:
            connected = True
            _set_pixel(0, 8, 0)  # green = connected
        elif not server.is_connected() and connected:
            connected = False
            _set_pixel(0, 0, 8)  # blue = back to advertising
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
