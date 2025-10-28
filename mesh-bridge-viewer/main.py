#!/usr/bin/env python3
# pip install bleak
import asyncio
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from datetime import datetime
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakDBusError
from collections.abc import Callable

BRIDGE_SERVICE = "0000fff0-0000-1000-8000-00805f9b34fb"
BRIDGE_CHAR    = "0000fff1-0000-1000-8000-00805f9b34fb"
BRIDGE_TX = "0000fff2-0000-1000-8000-00805f9b34fb"

AUTO_RECONNECT_DELAY_S = 2.0
SCAN_TIMEOUT_S = 6.0

async def bleak_is_connected(client) -> bool:
    """
    Works across Bleak versions:
    - If is_connected is a bool property, return it.
    - If it's a method, call it.
    - If it returns a coroutine, await it.
    """
    try:
        attr = client.is_connected
        # Property case
        if isinstance(attr, bool):
            return attr
        # Method case
        if isinstance(attr, Callable):
            res = attr()
            if asyncio.iscoroutine(res):
                return await res
            return bool(res)
    except Exception:
        pass
    return False

def printable_text(b: bytes) -> str:
    # Convert only printable ASCII 32–126; others as dots
    return ''.join(chr(x) if 32 <= x < 127 else '.' for x in b)


class BLEChatApp:
    def __init__(self, root, name_default="ESP-BLE-MESH"):
        self.root = root
        root.title("Mesh Bridge Viewer")
        root.geometry("640x540")

        top = tk.Frame(root)
        top.pack(fill=tk.X, padx=8, pady=6)

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

        self.chat_log = ScrolledText(root, state="disabled", width=80, height=24)
        self.chat_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0,8))

        bottom = tk.Frame(root)
        bottom.pack(fill=tk.X, padx=8, pady=(0,8))
        self.entry = tk.Entry(bottom)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind("<Return>", self.send_message)
        tk.Button(bottom, text="Send", command=self.send_message).pack(side=tk.LEFT, padx=8)

        # BLE state
        self.client: BleakClient | None = None
        self.ble_task: asyncio.Task | None = None
        self.running = True
        self.last_addr: str | None = None

        # kick off auto-connect on launch
        self.ble_task = asyncio.create_task(self.ble_supervisor())

        # Setup window close handler
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        root.after(10, self._poll_loop)

    def _on_close(self):
        self.running = False
        self.root.destroy()

    def _poll_loop(self):
        if not self.running:
            return
        self.root.after(10, self._poll_loop)

    # ---------- UI ----------
    def log_line(self, who: str, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.chat_log.config(state="normal")
        self.chat_log.insert(tk.END, f"[{ts}] {who}: {text}\n")
        self.chat_log.see(tk.END)
        self.chat_log.config(state="disabled")

    def send_message(self, _evt=None):
        msg = self.entry.get().strip()
        if not msg:
            return
        self.entry.delete(0, tk.END)
        self.log_line("You", msg)
        # Write to ESP (best-effort)
        if self.client:
            try:
                data = msg.encode("utf-8")[:90]  # keep under BRIDGE_VAL_MAX
                asyncio.create_task(self.client.write_gatt_char(BRIDGE_TX, data, response=False))
            except Exception as e:
                self.log_line("Error", f"Send failed: {e}")

    def user_connect(self):
        # manual connect/reconnect on button
        if self.ble_task and not self.ble_task.done():
            self.log_line("System", "Connect/reconnect already in progress.")
            return
        self.ble_task = asyncio.create_task(self.ble_supervisor())

    def status(self, s: str):
        self.status_var.set(s)

    def _poll_loop(self):
        if not self.running:
            return
        self.root.after(10, self._poll_loop)

    # ---------- BLE logic ----------
    async def ble_supervisor(self):
        """Auto-connect loop: discover -> connect -> subscribe -> wait; on drop, retry."""
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
                    # Block here until disconnected
                    while await bleak_is_connected(client):
                        await asyncio.sleep(0.3)
            except Exception as e:
                self.client = None
                self.status("Disconnected")
                self.log_line("Error", str(e))
            # auto-reconnect after a short delay
            await asyncio.sleep(AUTO_RECONNECT_DELAY_S)

    async def pick_address(self) -> str:
        """Return address using: last_addr → explicit MAC → service UUID → name substring. Retries until found."""
        # 1) If user typed MAC, use it
        mac = self.addr_var.get().strip()
        if mac:
            return mac
        # 2) If we had a successful address before, try it first
        if self.last_addr:
            return self.last_addr

        name_sub = (self.name_var.get() or "").strip() or None

        async def find_once():
            # Prefer matching by service UUID
            def match_service(d, adv):
                su = getattr(adv, "service_uuids", None) or []
                if BRIDGE_SERVICE in su:
                    return True if not name_sub else (d.name or "").lower().find(name_sub.lower()) != -1
                return False
            dev = await BleakScanner.find_device_by_filter(match_service, timeout=SCAN_TIMEOUT_S)
            if dev:
                return dev.address
            # Fallback: by name substring only (if you didn't advertise 0xFFF0)
            if name_sub:
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, adv: (d.name or "").lower().find(name_sub.lower()) != -1,
                    timeout=SCAN_TIMEOUT_S
                )
                if dev:
                    return dev.address
            return None

        # Retry until found; handle BlueZ "InProgress"
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

    def _on_notify(self, _sender, data: bytes):
        hexs = data.hex()
        text = printable_text(data)
        # marshal back to Tk thread
        self.root.after(0, self.log_line, "ESP-32", f"{hexs}    | {text}")

async def main_async():
    root = tk.Tk()
    app = BLEChatApp(root)
    while True:
        try:
            root.update()
        except tk.TclError:
            app.running = False
            break
        await asyncio.sleep(0.01)

def main():
    asyncio.run(main_async())

if __name__ == "__main__":
    main()
