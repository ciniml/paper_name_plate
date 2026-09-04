#!/usr/bin/env python3
"""Send a 1-bpp image to the PaperPlate over BLE (PC test harness).

Protocol v2: windowed, acknowledged transfer (see src/app/ble.rs).

    ble_send.py [chunk_bytes] [checker|border] [XX:XX:XX:XX:XX:XX]
    W=480 H=800 ACK_EVERY=8 ble_send.py
"""
import asyncio, os, struct, sys, time, zlib
from bleak import BleakClient, BleakScanner

SVC  = "50415045-5250-4c41-5445-000000000001"
CTRL = "50415045-5250-4c41-5445-000000000002"
DATA = "50415045-5250-4c41-5445-000000000003"
STAT = "50415045-5250-4c41-5445-000000000004"

ST = {0: "idle", 1: "receiving", 2: "committed", 3: "crc-error", 4: "incomplete"}


def make_image(w, h, kind):
    rb = (w + 7) // 8
    bits = bytearray(rb * h)

    def px(x, y):
        if 0 <= x < w and 0 <= y < h:
            bits[y * rb + (x >> 3)] |= 0x80 >> (x & 7)

    if kind == "checker":
        for y in range(h):
            for x in range(w):
                if ((x // 16) + (y // 16)) & 1:
                    px(x, y)
    elif kind == "border":
        for x in range(w):
            for t in range(4):
                px(x, t); px(x, h - 1 - t)
        for y in range(h):
            for t in range(4):
                px(t, y); px(w - 1 - t, y)
        for i in range(min(w, h)):
            px(i * w // h, i); px(w - 1 - i * w // h, i)
    return bits


class Status:
    """Collects STATUS notifications; lets the sender wait for a condition."""

    def __init__(self):
        self.q = asyncio.Queue()
        self.last = None

    def on_notify(self, _h, data):
        st = struct.unpack("<BII", bytes(data[:9]))
        self.last = st
        self.q.put_nowait(st)

    async def wait(self, pred, timeout):
        """Wait until a notification satisfying pred arrives; None on timeout."""
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            try:
                st = await asyncio.wait_for(self.q.get(), remain)
            except asyncio.TimeoutError:
                return None
            if pred(st):
                return st


async def main():
    w = int(os.environ.get("W", "480")); h = int(os.environ.get("H", "800"))
    ack_every = int(os.environ.get("ACK_EVERY", "8"))
    # This PC has two adapters; only hci1 reaches the plate reliably.
    adapter = os.environ.get("BLE_ADAPTER", "hci1")
    chunk = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 0
    kind = sys.argv[2] if len(sys.argv) > 2 else "checker"
    bits = make_image(w, h, kind)
    crc = zlib.crc32(bits) & 0xFFFFFFFF
    print(f"image {w}x{h} = {len(bits)} B crc32={crc:08x} kind={kind} ack_every={ack_every}")

    addr = next((a for a in sys.argv if a.count(":") == 5), None)
    if addr:
        target = addr
    else:
        print("scanning for PaperPlate (persistent)...")
        dev = None
        deadline = time.time() + 60
        while time.time() < deadline:
            dev = await BleakScanner.find_device_by_name("PaperPlate", timeout=6, adapter=adapter)
            if dev:
                break
            # BlueZ sometimes stops re-reporting a device it already knows;
            # a plain discovery pass still lists it.
            for d in await BleakScanner.discover(timeout=4, adapter=adapter):
                if d.name == "PaperPlate":
                    dev = d
                    break
            if dev:
                break
        if not dev:
            print("NOT FOUND"); return
        target = dev
    print("connecting", target)
    async with BleakClient(target, adapter=adapter) as c:
        try:
            await c._backend._acquire_mtu()
        except Exception:
            pass
        mtu = getattr(c, "mtu_size", 23)
        print("MTU:", mtu)
        if chunk == 0:
            chunk = max(16, min(mtu - 3, 125) - 2)  # ATT payload minus 2-byte offset header
        print("chunk:", chunk)

        status = Status()
        await c.start_notify(STAT, status.on_notify)

        start = struct.pack("<BHHIIB", 0x10, w, h, len(bits), crc, ack_every)
        await c.write_gatt_char(CTRL, start, response=True)
        st = await status.wait(lambda s: s[0] == 1, 2.0)
        print("start ack:", st)
        if st is None:
            print("no start notification; aborting"); return

        t0 = time.time()
        off = 0
        rewinds = 0
        while off < len(bits):
            burst_start = off
            for _ in range(ack_every):
                if off >= len(bits):
                    break
                part = bits[off:off + chunk]
                await c.write_gatt_char(DATA, struct.pack("<H", off) + part, response=False)
                off += len(part)
            burst_end = off
            st = await status.wait(lambda s: s[1] >= burst_end, 1.0)
            if st is None:
                # Ask the device where it really is.
                await c.write_gatt_char(CTRL, bytes([0x04]), response=True)
                st = await status.wait(lambda s: True, 1.0)
                if st is None:
                    raw = await c.read_gatt_char(STAT)
                    st = struct.unpack("<BII", bytes(raw[:9]))
                if st[1] < burst_end:
                    rewinds += 1
                    print(f"  rewind {burst_end} -> {st[1]} (burst started at {burst_start})")
                    off = st[1]
            if off % 4800 < chunk * ack_every:
                print(f"  {off}/{len(bits)} ({off * 100 // len(bits)}%) {time.time() - t0:.1f}s")
        dt = time.time() - t0
        print(f"sent {len(bits)} B in {dt:.1f}s ({len(bits) / dt / 1024:.1f} KiB/s), rewinds={rewinds}")

        await c.write_gatt_char(CTRL, bytes([0x02]), response=False)
        st = await status.wait(lambda s: s[0] != 1, 3.0)
        if st is None:
            raw = await c.read_gatt_char(STAT)
            st = struct.unpack("<BII", bytes(raw[:9]))
        print(f"commit -> {ST.get(st[0], st[0])} received={st[1]} expected={st[2]}")
        await c.stop_notify(STAT)


asyncio.run(main())
