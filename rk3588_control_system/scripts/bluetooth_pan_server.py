#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / "config" / "bluetooth.config.json"
RUN_DIR = Path("/run")
NAP_UUID = "00001116-0000-1000-8000-00805f9b34fb"
AGENT_PATH = "/manta/agent"


def run(cmd, check=True):
    print("[bt-pan] " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=check, text=True)


def output(cmd):
    return subprocess.run(cmd, check=False, text=True, capture_output=True).stdout


def load_config():
    cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    pan = cfg.get("pan", {})
    return {
        "enabled": bool(pan.get("enabled", True)),
        "device_name": str(pan.get("device_name", cfg.get("bluetooth", {}).get("device_name", "Manta-Control"))),
        "bridge": str(pan.get("bridge", "manta-bt0")),
        "ip_address": str(pan.get("ip_address", "10.43.0.1")),
        "cidr": str(pan.get("cidr", "10.43.0.1/24")),
        "dhcp_start": str(pan.get("dhcp_start", "10.43.0.20")),
        "dhcp_end": str(pan.get("dhcp_end", "10.43.0.80")),
        "discoverable": bool(pan.get("discoverable", True)),
        "pairable": bool(pan.get("pairable", True)),
        "pin": str(os.environ.get("MANTA_BLUETOOTH_PIN") or cfg.get("security", {}).get("pin", "CHANGE_ME_AT_INSTALL")),
    }


def ensure_bridge(name, cidr):
    if subprocess.run(["ip", "link", "show", name], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
        run(["ip", "link", "add", name, "type", "bridge"])
    run(["ip", "addr", "flush", "dev", name], check=False)
    run(["ip", "addr", "add", cidr, "dev", name])
    run(["ip", "link", "set", name, "up"])


def stop_existing_dnsmasq(pidfile):
    try:
        pid = int(Path(pidfile).read_text(encoding="utf-8").strip())
    except Exception:
        return
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_text(errors="ignore")
    except Exception:
        return
    if "dnsmasq" not in cmdline or pidfile not in cmdline:
        return
    print(f"[bt-pan] stopping stale dnsmasq pid={pid}", flush=True)
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def configure_adapter(bus, cfg):
    adapter = dbus.Interface(
        bus.get_object("org.bluez", "/org/bluez/hci0"),
        "org.freedesktop.DBus.Properties",
    )
    adapter.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(True))
    adapter.Set("org.bluez.Adapter1", "Alias", dbus.String(cfg["device_name"]))
    adapter.Set("org.bluez.Adapter1", "PairableTimeout", dbus.UInt32(0))
    adapter.Set("org.bluez.Adapter1", "DiscoverableTimeout", dbus.UInt32(0))
    adapter.Set("org.bluez.Adapter1", "Pairable", dbus.Boolean(cfg["pairable"]))
    adapter.Set("org.bluez.Adapter1", "Discoverable", dbus.Boolean(cfg["discoverable"]))


def register_network_server(bus, bridge):
    server = dbus.Interface(
        bus.get_object("org.bluez", "/org/bluez/hci0"),
        "org.bluez.NetworkServer1",
    )
    try:
        server.Unregister(NAP_UUID)
    except Exception:
        pass
    server.Register(NAP_UUID, bridge)
    print(f"[bt-pan] registered NAP on {bridge}", flush=True)
    return server


class MantaAgent(dbus.service.Object):
    def __init__(self, bus, pin):
        super().__init__(bus, AGENT_PATH)
        self.pin = pin

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Release(self):
        print("[bt-pan] pairing agent released", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        print(f"[bt-pan] pairing PIN requested by {device}", flush=True)
        return self.pin

    @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
    def DisplayPasskey(self, device, passkey):
        print(f"[bt-pan] display passkey {passkey:06d} for {device}", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        print(f"[bt-pan] display PIN {pincode} for {device}", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        print(f"[bt-pan] passkey requested by {device}", flush=True)
        try:
            return dbus.UInt32(int(self.pin))
        except ValueError:
            return dbus.UInt32(0)

    @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        print(f"[bt-pan] accepting passkey {passkey:06d} from {device}", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        print(f"[bt-pan] authorizing {device}", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        print(f"[bt-pan] authorizing service {uuid} for {device}", flush=True)

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Cancel(self):
        print("[bt-pan] pairing request canceled", flush=True)


def register_agent(bus, pin):
    agent = MantaAgent(bus, pin)
    manager = dbus.Interface(
        bus.get_object("org.bluez", "/org/bluez"),
        "org.bluez.AgentManager1",
    )
    try:
        manager.UnregisterAgent(AGENT_PATH)
    except Exception:
        pass
    manager.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
    manager.RequestDefaultAgent(AGENT_PATH)
    print("[bt-pan] registered pairing agent", flush=True)
    return manager, agent


def main():
    cfg = load_config()
    if not cfg["enabled"]:
        print("[bt-pan] disabled in config", flush=True)
        return 0

    ensure_bridge(cfg["bridge"], cfg["cidr"])

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    configure_adapter(bus, cfg)
    agent_manager, _agent = register_agent(bus, cfg["pin"])
    server = register_network_server(bus, cfg["bridge"])

    pidfile = str(RUN_DIR / "manta-bluetooth-pan-dnsmasq.pid")
    stop_existing_dnsmasq(pidfile)
    dnsmasq = subprocess.Popen(
        [
            "dnsmasq",
            "--keep-in-foreground",
            "--conf-file=/dev/null",
            f"--interface={cfg['bridge']}",
            "--bind-interfaces",
            f"--listen-address={cfg['ip_address']}",
            f"--dhcp-range={cfg['dhcp_start']},{cfg['dhcp_end']},255.255.255.0,12h",
            f"--dhcp-option=3,{cfg['ip_address']}",
            f"--dhcp-option=6,{cfg['ip_address']}",
            f"--pid-file={pidfile}",
        ],
        text=True,
    )

    loop = GLib.MainLoop()

    def shutdown(*_args):
        print("[bt-pan] shutting down", flush=True)
        try:
            server.Unregister(NAP_UUID)
        except Exception:
            pass
        try:
            agent_manager.UnregisterAgent(AGENT_PATH)
        except Exception:
            pass
        dnsmasq.terminate()
        loop.quit()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f"[bt-pan] ready: pair with {cfg['device_name']} and open http://{cfg['ip_address']}:3000", flush=True)
    loop.run()
    dnsmasq.wait(timeout=3)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
