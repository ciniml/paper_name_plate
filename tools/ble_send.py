#!/usr/bin/env python3
"""Send a 1-bpp image to the PaperPlate over BLE (PC test harness)."""
import asyncio, struct, sys, time
from bleak import BleakClient, BleakScanner

SVC  = "50415045-5250-4c41-5445-000000000001"
CTRL = "50415045-5250-4c41-5445-000000000002"
DATA = "50415045-5250-4c41-5445-000000000003"
STAT = "50415045-5250-4c41-5445-000000000004"

def make_image(w, h, kind):
    rb = (w + 7) // 8
    bits = bytearray(rb * h)
    def px(x, y):
        if 0 <= x < w and 0 <= y < h:
            bits[y*rb + (x >> 3)] |= 0x80 >> (x & 7)
    if kind == "checker":
        for y in range(h):
            for x in range(w):
                if ((x // 16) + (y // 16)) & 1:
                    px(x, y)
    elif kind == "border":
        for x in range(w):
            for t in range(4):
                px(x, t); px(x, h-1-t)
        for y in range(h):
            for t in range(4):
                px(t, y); px(w-1-t, y)
        for i in range(min(w, h)):
            px(i * w // h, i); px(w-1 - i * w // h, i)
    return bits

async def main():
    import os
    w = int(os.environ.get("W", "480")); h = int(os.environ.get("H", "800"))
    chunk = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    kind = sys.argv[2] if len(sys.argv) > 2 else "checker"
    bits = make_image(w, h, kind)
    print(f"image {w}x{h} = {len(bits)} B, chunk={chunk}, kind={kind}")

    addr = None
    for a in sys.argv:
        if a.count(":") == 5:
            addr = a
    if addr:
        target = addr
    else:
        print("scanning for PaperPlate (persistent)...")
        import time as _t
        dev = None
        deadline = _t.time() + 60
        while _t.time() < deadline:
            dev = await BleakScanner.find_device_by_name("PaperPlate", timeout=6)
            if dev:
                break
        if not dev:
            print("NOT FOUND"); return
        target = dev
    print("connecting", target)
    async with BleakClient(target) as c:
        try:
            await c._backend._acquire_mtu()
        except Exception:
            pass
        print("MTU:", getattr(c, "mtu_size", "?"))
        start = struct.pack("<BHHI", 1, w, h, len(bits))
        await c.write_gatt_char(CTRL, start, response=True)
        t0 = time.time()
        off = 0
        n = 0
        while off < len(bits):
            part = bits[off:off+chunk]
            # Write Command (no response) is far faster than Write Request;
            # pace every 8 writes so the controller's ACL buffers don't overflow.
            await c.write_gatt_char(DATA, part, response=False)
            off += len(part)
            n += 1
            if n % 8 == 0:
                await asyncio.sleep(0.02)
            if off % 4800 < chunk:
                print(f"  {off}/{len(bits)} ({off*100//len(bits)}%)")
        # Let the last commands drain before checking.
        await asyncio.sleep(0.3)
        dt = time.time() - t0
        st = await c.read_gatt_char(STAT)
        state, recv, exp = struct.unpack("<BII", st[:9])
        print(f"sent {len(bits)} B in {dt:.1f}s; device state={state} received={recv} expected={exp}")
        if recv == len(bits):
            # No-response: the device blocks on the (slow) EPD refresh and
            # would miss an ATT write-response, so don't wait for one.
            await c.write_gatt_char(CTRL, bytes([2]), response=False)
            print("COMMIT sent")
        else:
            await c.write_gatt_char(CTRL, bytes([3]), response=False)
            print("MISMATCH -> abort")
        await asyncio.sleep(0.5)

asyncio.run(main())
