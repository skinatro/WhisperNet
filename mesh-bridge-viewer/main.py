#!/usr/bin/env python3
# pip install bleak
import asyncio
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from datetime import datetime
from collections.abc import Callable
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakDBusError

BRIDGE_SERVICE = "0000fff0-0000-1000-8000-00805f9b34fb"
BRIDGE_CHAR    = "0000fff1-0000-1000-8000-00805f9b34fb"  # notify (ESP -> PC)
BRIDGE_TX      = "0000fff2-0000-1000-8000-00805f9b34fb"  # write  (PC -> ESP)

AUTO_RECONNECT_DELAY_S = 2.0
SCAN_TIMEOUT_S = 6.0

# ---- ultra-light "encryption" (Caesar shift over bytes) ----
CX_HEADER = b"CX1"  # 3-byte marker so we know when to decrypt

def caesar_shift(buf: bytes, shift: int) -> bytes:
    s = shift % 256
    return bytes((b + s) & 0xFF for b in buf)

def printable_text(b: bytes) -> str:
    return ''.join(chr(x) if 32 <= x < 127 else '.' for x in b)

async def bleak_is_connected(client) -> bool:
    try:
        attr = client.is_connected
        if isinstance(attr, bool):
            return attr
        if isinstance(attr, Callable):
            res = attr()
            if asyncio.iscoroutine(res):
                return await res
            return bool(res)
    except Exception:
        pass
    return False

class BLEChatApp:
    def __init__(self, root, name_default="ESP-BLE-MESH"):
        self.root = root
        root.title("Mesh Bridge Viewer (Caesar demo)")
        root.geometry("760x600")

        # ---- top row: discovery/connect
        top = tk.Frame(root); top.pack(fill=tk.X, padx=8, pady=6)
        tk.Label(top, text="Name contains:").pack(side=tk.LEFT)
        self.name_var = tk.StringVar(value=name_default)
        tk.Entry(top, textvariable=self.name_var, width=20).pack(side=tk.LEFT, padx=6)

        tk.Label(top, text="MAC (optional):").pack(side=tk.LEFT)
        self.addr_var = tk.StringVar(value="")
        tk.Entry(top, textvariable=self.addr_var, width=20).pack(side=tk.LEFT, padx=6)

        self.connect_btn = tk.Button(top, text="Connect", command=self.user_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_var = tk.StringVar(value="Idle")
        tk.Label(top, textvariable=self.status_var, fg="#666").pack(side=tk.RIGHT)

        # ---- crypto row: offset + toggles
        crypto = tk.Frame(root); crypto.pack(fill=tk.X, padx=8, pady=(0,6))
        tk.Label(crypto, text="Shift offset (−128..127):").pack(side=tk.LEFT)
        self.shift_var = tk.StringVar(value="1")   # default +1
        tk.Entry(crypto, textvariable=self.shift_var, width=8).pack(side=tk.LEFT, padx=6)

        self.encrypt_out = tk.BooleanVar(value=True)
        self.decrypt_in  = tk.BooleanVar(value=True)
        tk.Checkbutton(crypto, text="Encrypt outgoing", variable=self.encrypt_out).pack(side=tk.LEFT, padx=8)
        tk.Checkbutton(crypto, text="Decrypt incoming", variable=self.decrypt_in).pack(side=tk.LEFT, padx=8)

        # ---- chat log
        self.chat_log = ScrolledText(root, state="disabled", width=92, height=26)
        self.chat_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0,8))

        # ---- bottom: entry/send
        bottom = tk.Frame(root); bottom.pack(fill=tk.X, padx=8, pady=(0,8))
        self.entry = tk.Entry(bottom)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind("<Return>", self.send_message)
        tk.Button(bottom, text="Send", command=self.send_message).pack(side=tk.LEFT, padx=8)

        # BLE state
        self.client: BleakClient | None = None
        self.ble_task: asyncio.Task | None = None
        self.running = True
        self.last_addr: str | None = None

        # start auto-connect loop
        self.ble_task = asyncio.create_task(self.ble_supervisor())

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(10, self._poll_loop)

    # ---------- plumbing ----------
    def _on_close(self):
        self.running = False
        self.root.destroy()

    def _poll_loop(self):
        if not self.running:
            return
        self.root.after(10, self._poll_loop)

    def log_line(self, who: str, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.chat_log.config(state="normal")
        self.chat_log.insert(tk.END, f"[{ts}] {who}: {text}\n")
        self.chat_log.see(tk.END)
        self.chat_log.config(state="disabled")

    def status(self, s: str):
        self.status_var.set(s)

    # ---------- send ----------
    def _get_shift(self) -> int:
        try:
            val = int(self.shift_var.get().strip())
        except Exception:
            val = 0
        # clamp to signed byte range for clarity (wrap anyway in function)
        if val < -128: val = -128
        if val > 127:  val = 127
        return val

    def send_message(self, _evt=None):
        msg = self.entry.get().strip()
        if not msg:
            return
        self.entry.delete(0, tk.END)

        raw = msg.encode("utf-8")
        out = raw
        label = "You"

        if self.encrypt_out.get():
            shift = self._get_shift()
            out = CX_HEADER + caesar_shift(raw, shift)
            label = f"You (enc s={shift})"

        if self.client:
            try:
                if len(out) > 180:  # keep well under typical MTU-fragment comfort
                    self.log_line("System", f"Message too long ({len(out)} bytes). Truncating to 180.")
                    out = out[:180]
                asyncio.create_task(self.client.write_gatt_char(BRIDGE_TX, out, response=False))
            except Exception as e:
                self.log_line("Error", f"Send failed: {e}")
        else:
            self.log_line("Error", "Not connected; cannot send.")

        self.log_line(label, msg)

    # ---------- BLE supervisor ----------
    def user_connect(self):
        if self.ble_task and not self.ble_task.done():
            self.log_line("System", "Connect/reconnect already in progress.")
            return
        self.ble_task = asyncio.create_task(self.ble_supervisor())

    async def ble_supervisor(self):
        while self.running:
            try:
                addr = await self.pick_address()
                self.status(f"Connecting to {addr}…")
                self.log_line("System", f"Connecting to {addr}…")
                async with BleakClient(addr) as client:
                    self.client = client
                    self.last_addr = addr
                    await client.start_notify(BRIDGE_CHAR, self._on_notify)
                    self.status("Connected. Listening.")
                    self.log_line("System", "Connected. Listening for bridge notifications…")
                    while await bleak_is_connected(client):
                        await asyncio.sleep(0.3)
            except Exception as e:
                self.client = None
                self.status("Disconnected")
                self.log_line("Error", str(e))
            await asyncio.sleep(AUTO_RECONNECT_DELAY_S)

    async def pick_address(self) -> str:
        mac = self.addr_var.get().strip()
        if mac:
            return mac
        if self.last_addr:
            return self.last_addr

        name_sub = (self.name_var.get() or "").strip() or None

        async def find_once():
            def match_service(d, adv):
                su = getattr(adv, "service_uuids", None) or []
                if BRIDGE_SERVICE in su:
                    return True if not name_sub else (d.name or "").lower().find(name_sub.lower()) != -1
                return False
            dev = await BleakScanner.find_device_by_filter(match_service, timeout=SCAN_TIMEOUT_S)
            if dev:
                return dev.address
            if name_sub:
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, adv: (d.name or "").lower().find(name_sub.lower()) != -1,
                    timeout=SCAN_TIMEOUT_S
                )
                if dev:
                    return dev.address
            return None

        attempt = 0
        while True:
            attempt += 1
            self.status(f"Scanning… (try {attempt})")
            try:
                addr = await find_once()
                if addr:
                    self.log_line("System", f"Selected [{addr}]")
                    return addr
                self.log_line("System", "No device found; rescanning…")
            except BleakDBusError as e:
                if "InProgress" in str(e):
                    self.log_line("System", "BlueZ scan busy; retrying…")
                else:
                    raise
            await asyncio.sleep(1.0)

    # ---------- notifications ----------
    def _on_notify(self, _sender, data: bytes):
        line: str
        if self.decrypt_in.get() and data.startswith(CX_HEADER):
            body = data[len(CX_HEADER):]
            shift = self._get_shift()
            dec  = caesar_shift(body, -shift)
            text = dec.decode("utf-8", errors="replace")
            line = f"(dec s={shift}) {text}"
        else:
            hexs = data.hex()
            prnt = printable_text(data)
            line = f"{hexs}    | {prnt}"
        self.root.after(0, self.log_line, "ESP-32", line)

# ---- app loop ----
async def main_async():
    root = tk.Tk()
    app = BLEChatApp(root)
    while True:
        try:
            root.update()
        except tk.TclError:
            break
        await asyncio.sleep(0.01)

def main():
    asyncio.run(main_async())

if __name__ == "__main__":
    main()
