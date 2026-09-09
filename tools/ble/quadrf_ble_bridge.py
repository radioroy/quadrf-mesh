#!/usr/bin/env python3
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib
import logging
import socket
import struct
import subprocess
import threading
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("quadrf-ble")

BLUEZ_SERVICE_NAME = "org.bluez"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"
ADAPTER_IFACE = "org.bluez.Adapter1"
AGENT_IFACE = "org.bluez.Agent1"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"
ADAPTER_PATH = "/org/bluez/hci0"
AGENT_PATH = "/org/bluez/quadrf/agent"

# Standard Meshtastic BLE UUIDs
MESHTASTIC_SERVICE_UUID = "6ba1b218-15a8-461f-9fa8-5dcae273eafd"
TORADIO_UUID = "f75c76d2-129e-4dad-a1dd-7866124401e7"
FROMRADIO_UUID = "2c55e69e-4993-11ed-b878-0242ac120002"
FROMNUM_UUID = "ed9da18c-a800-4f66-a670-aa7547e34453"

# Unprogrammed BCM ROM / patch-ram placeholders. Android will not
# complete a GATT connect to AA:AA:AA:AA:AA:AA.
PLACEHOLDER_BDADDRS = {
    "AA:AA:AA:AA:AA:AA",
    "00:00:00:00:00:00",
    "43:45:C0:00:1F:AC",
}

# ToRadio.heartbeat is an empty protobuf submessage at field 7.
HEARTBEAT_PROTO = b"\x3a\x00"
HEARTBEAT_SEC = 30


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.InvalidArguments"

class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


def advertised_name():
    host = socket.gethostname().split(".")[0]
    try:
        with open("/etc/quadrf/quadrf.conf", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("QUADRF_HOSTNAME="):
                    val = line.split("=", 1)[1].strip().strip("'\"")
                    if val:
                        host = val.split(".")[0]
                        break
    except OSError:
        pass
    name = f"QuadRF-{host.upper()}"
    return name[:16]


def btmgmt(*args):
    # systemd units have no TTY; btmgmt then blocks on the mgmt socket.
    # script(1) gives it a PTY so the command complete event is delivered.
    cmdline = "btmgmt --index 0 " + " ".join(args)
    cmd = ["script", "-q", "-c", cmdline, "/dev/null"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
    except (OSError, subprocess.TimeoutExpired) as e:
        logger.warning("btmgmt %s failed: %s", " ".join(args), e)
        return False
    text = (r.stderr or r.stdout).strip()
    if r.returncode != 0 or "failed" in text.lower() or "rejected" in text.lower():
        logger.warning("btmgmt %s: %s", " ".join(args), text)
        return False
    return True


def wait_for_hci(bus, timeout=12.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            bus.get_object(BLUEZ_SERVICE_NAME, ADAPTER_PATH)
            return
        except dbus.exceptions.DBusException:
            time.sleep(0.2)
    raise RuntimeError("BlueZ adapter hci0 not available")


def prepare_adapter(bus, local_name):
    """Make hci0 a connectable LE peripheral with a unique public address."""
    wait_for_hci(bus)
    adapter_obj = bus.get_object(BLUEZ_SERVICE_NAME, ADAPTER_PATH)
    props = dbus.Interface(adapter_obj, DBUS_PROP_IFACE)
    addr = str(props.Get(ADAPTER_IFACE, "Address")).upper()
    logger.info("Adapter address at start: %s", addr)

    if addr in PLACEHOLDER_BDADDRS:
        # public-addr needs a power cycle, and power off deadlocks against
        # bluetoothd AutoEnable. Install bluez-firmware and reboot so the
        # BCM4345C0 patch-ram can program the OTP address.
        logger.error(
            "Placeholder BDADDR %s; install bluez-firmware and reboot. "
            "Android will not connect to AA:AA:AA:AA:AA:AA.",
            addr,
        )

    # Do not btmgmt power off here: bluetoothd holds the adapter and the
    # command hangs. connectable/bondable work while powered.
    btmgmt("connectable", "on")
    btmgmt("bondable", "on")
    btmgmt("discov", "on")

    props = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, ADAPTER_PATH), DBUS_PROP_IFACE
    )
    try:
        props.Set(ADAPTER_IFACE, "Alias", dbus.String(local_name))
        props.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(True))
        props.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(True))
        props.Set(ADAPTER_IFACE, "DiscoverableTimeout", dbus.UInt32(0))
    except dbus.exceptions.DBusException as e:
        logger.warning("Adapter property set failed: %s", e)

    addr = str(props.Get(ADAPTER_IFACE, "Address")).upper()
    pairable = bool(props.Get(ADAPTER_IFACE, "Pairable"))
    logger.info("Adapter ready: addr=%s pairable=%s name=%s", addr, pairable, local_name)
    if addr in PLACEHOLDER_BDADDRS:
        logger.error("Adapter still has placeholder BDADDR %s; phones will fail to connect", addr)


class Agent(dbus.service.Object):
    """Just-Works agent so the phone can bond without a passkey display."""

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        logger.info("AuthorizeService %s %s", device, uuid)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        logger.info("RequestPinCode %s", device)
        return "000000"

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        logger.info("RequestPasskey %s", device)
        return dbus.UInt32(0)

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def DisplayPasskey(self, device, passkey):
        logger.info("DisplayPasskey %s %06d", device, passkey)

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        logger.info("DisplayPinCode %s %s", device, pincode)

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        logger.info("RequestConfirmation %s passkey=%06d (auto-accept)", device, passkey)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        logger.info("RequestAuthorization %s", device)

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Cancel(self):
        logger.info("Agent Cancel")

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        logger.info("Agent Release")


def register_agent(bus):
    agent = Agent(bus, AGENT_PATH)
    mgr = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, "/org/bluez"), AGENT_MANAGER_IFACE
    )
    mgr.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
    mgr.RequestDefaultAgent(AGENT_PATH)
    logger.info("Registered NoInputNoOutput pairing agent")
    return agent


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = f"{service.path}/char{index}"
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.notifying = False
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": self.flags,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        logger.info("StartNotify %s", self.uuid)
        self.notifying = True

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        logger.info("StopNotify %s", self.uuid)
        self.notifying = False

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

class ToRadioCharacteristic(Characteristic):
    def __init__(self, bus, index, service, tcp_bridge):
        super().__init__(bus, index, TORADIO_UUID, ["write", "write-without-response"], service)
        self.tcp_bridge = tcp_bridge

    def WriteValue(self, value, options):
        data = bytes(value)
        logger.info("ToRadio write %d bytes", len(data))
        self.tcp_bridge.send_to_radio(data)

class FromRadioCharacteristic(Characteristic):
    def __init__(self, bus, index, service, tcp_bridge):
        super().__init__(bus, index, FROMRADIO_UUID, ["read"], service)
        self.tcp_bridge = tcp_bridge

    def ReadValue(self, options):
        pkt = self.tcp_bridge.pop_from_radio()
        if pkt:
            logger.info("FromRadio read %d bytes (%d queued)", len(pkt), self.tcp_bridge.queue_len())
        return dbus.ByteArray(pkt)

class FromNumCharacteristic(Characteristic):
    def __init__(self, bus, index, service, tcp_bridge):
        super().__init__(bus, index, FROMNUM_UUID, ["read", "notify"], service)
        self.tcp_bridge = tcp_bridge
        self.tcp_bridge.set_notify_callback(self.on_new_packet)

    def ReadValue(self, options):
        val = struct.pack("<I", self.tcp_bridge.from_num & 0xFFFFFFFF)
        return dbus.ByteArray(val)

    def on_new_packet(self, num):
        if self.notifying:
            val = struct.pack("<I", num & 0xFFFFFFFF)
            self.PropertiesChanged(
                GATT_CHRC_IFACE,
                {"Value": dbus.ByteArray(val)},
                []
            )

class Service(dbus.service.Object):
    def __init__(self, bus, index, uuid, primary):
        self.path = f"/org/bluez/quadrf/service{index}"
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": self.primary,
                "Characteristics": dbus.Array(
                    [c.get_path() for c in self.characteristics],
                    signature="o"
                )
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_SERVICE_IFACE]

class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = "/org/bluez/quadrf"
        self.services = []
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.characteristics:
                response[chrc.get_path()] = chrc.get_properties()
        return response

class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/bluez/quadrf/advertisement"

    def __init__(self, bus, index, local_name):
        self.path = f"{self.PATH_BASE}{index}"
        self.bus = bus
        self.ad_type = "peripheral"
        self.service_uuids = [MESHTASTIC_SERVICE_UUID]
        self.local_name = local_name
        self.include_tx_power = True
        super().__init__(bus, self.path)

    def get_properties(self):
        properties = {
            "Type": self.ad_type,
            "ServiceUUIDs": dbus.Array(self.service_uuids, signature="s"),
            "LocalName": dbus.String(self.local_name),
            "IncludeTxPower": dbus.Boolean(self.include_tx_power),
            "Discoverable": dbus.Boolean(True),
        }
        return {LE_ADVERTISEMENT_IFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        logger.info("Advertisement released")

class TcpBridge:
    def __init__(self, host="127.0.0.1", port=4403):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.rx_queue = []
        self.from_num = 0
        self.notify_cb = None
        self.running = True
        self.thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.thread.start()

    def set_notify_callback(self, cb):
        self.notify_cb = cb

    def queue_len(self):
        with self.lock:
            return len(self.rx_queue)

    def send_heartbeat(self):
        self.send_to_radio(HEARTBEAT_PROTO, quiet=True)
        return True

    def send_to_radio(self, data: bytes, quiet=False):
        with self.lock:
            if not self.sock:
                if not quiet:
                    logger.warning("Dropped toRadio write: TCP connection offline")
                return
            # Framing: 0x94 0xc3 + 2 bytes big-endian length + payload
            header = struct.pack(">BBH", 0x94, 0xC3, len(data))
            try:
                self.sock.sendall(header + data)
            except Exception as e:
                logger.error(f"TCP write error: {e}")
                self._disconnect()

    def pop_from_radio(self) -> bytes:
        with self.lock:
            if self.rx_queue:
                return self.rx_queue.pop(0)
            return b""

    def _disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _connect(self):
        while self.running:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                with self.lock:
                    self.sock = s
                logger.info(f"Connected to meshtasticd PhoneAPI on {self.host}:{self.port}")
                return s
            except Exception as e:
                logger.debug(f"Waiting for meshtasticd on {self.host}:{self.port}: {e}")
                time.sleep(2.0)

    def _rx_worker(self):
        while self.running:
            sock = self.sock
            if not sock:
                sock = self._connect()
                if not sock:
                    continue

            try:
                # Read 4-byte frame header: 0x94 0xc3 + uint16 big endian length
                hdr = bytearray()
                while len(hdr) < 4:
                    chunk = sock.recv(4 - len(hdr))
                    if not chunk:
                        raise ConnectionResetError("meshtasticd closed connection")
                    hdr.extend(chunk)

                if hdr[0] != 0x94 or hdr[1] != 0xC3:
                    logger.warning(f"Bad sync bytes: {hdr[0]:#x} {hdr[1]:#x}, discarding")
                    continue

                length = (hdr[2] << 8) | hdr[3]
                payload = bytearray()
                while len(payload) < length:
                    chunk = sock.recv(length - len(payload))
                    if not chunk:
                        raise ConnectionResetError("Premature EOF in payload")
                    payload.extend(chunk)

                with self.lock:
                    self.rx_queue.append(bytes(payload))
                    if len(self.rx_queue) > 64:
                        self.rx_queue.pop(0)
                    self.from_num += 1
                    num = self.from_num

                if self.notify_cb:
                    GLib.idle_add(self.notify_cb, num)

            except Exception as e:
                logger.warning(f"TCP socket error: {e}")
                with self.lock:
                    self._disconnect()
                time.sleep(1.0)

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    local_name = advertised_name()
    prepare_adapter(bus, local_name)
    agent = register_agent(bus)

    tcp_bridge = TcpBridge(host="127.0.0.1", port=4403)

    app = Application(bus)
    service = Service(bus, 0, MESHTASTIC_SERVICE_UUID, True)
    service.add_characteristic(ToRadioCharacteristic(bus, 0, service, tcp_bridge))
    service.add_characteristic(FromRadioCharacteristic(bus, 1, service, tcp_bridge))
    service.add_characteristic(FromNumCharacteristic(bus, 2, service, tcp_bridge))
    app.add_service(service)

    ad = Advertisement(bus, 0, local_name)

    adapter_obj = bus.get_object(BLUEZ_SERVICE_NAME, ADAPTER_PATH)
    gatt_mgr = dbus.Interface(adapter_obj, GATT_MANAGER_IFACE)
    adv_mgr = dbus.Interface(adapter_obj, LE_ADVERTISING_MANAGER_IFACE)
    mainloop = GLib.MainLoop()

    def app_reg_cb():
        logger.info("GATT Application registered with BlueZ")

    def app_reg_err_cb(error):
        logger.error(f"Failed to register GATT Application: {error}")
        mainloop.quit()

    def adv_reg_cb():
        logger.info(f"BLE Advertisement registered (Name={local_name})")

    def adv_reg_err_cb(error):
        logger.error(f"Failed to register BLE Advertisement: {error}")
        mainloop.quit()

    gatt_mgr.RegisterApplication(app.get_path(), {},
                                 reply_handler=app_reg_cb,
                                 error_handler=app_reg_err_cb)

    adv_mgr.RegisterAdvertisement(ad.get_path(), {},
                                  reply_handler=adv_reg_cb,
                                  error_handler=adv_reg_err_cb)

    GLib.timeout_add_seconds(HEARTBEAT_SEC, tcp_bridge.send_heartbeat)
    try:
        mainloop.run()
    except KeyboardInterrupt:
        logger.info("Terminating...")
    finally:
        tcp_bridge.running = False
        try:
            adv_mgr.UnregisterAdvertisement(ad.get_path())
        except Exception:
            pass
        try:
            gatt_mgr.UnregisterApplication(app.get_path())
        except Exception:
            pass
        try:
            dbus.Interface(
                bus.get_object(BLUEZ_SERVICE_NAME, "/org/bluez"),
                AGENT_MANAGER_IFACE,
            ).UnregisterAgent(AGENT_PATH)
        except Exception:
            pass
        del agent

if __name__ == "__main__":
    main()
