#!/usr/bin/env python3
"""Read or write the plate text over BLE (PC test harness).

    ble_content.py                      # read current text
    ble_content.py "Name" "Title" "Org" "Note" "example.com/x"   # write
"""
import asyncio, os, struct, sys, time, zlib
from bleak import BleakClient, BleakScanner

CTRL = "50415045-5250-4c41-5445-000000000002"
DATA = "50415045-5250-4c41-5445-000000000003"
STAT = "50415045-5250-4c41-5445-000000000004"
CONTENT = "50415045-5250-4c41-5445-000000000005"
ST = {0: "idle", 1: "receiving", 2: "committed", 3: "crc-error", 4: "incomplete"}


async def main():
    adapter = os.environ.get("BLE_ADAPTER", "hci1")
    dev = None
    for _ in range(8):
        dev = await BleakScanner.find_device_by_name("PaperPlate", timeout=6, adapter=adapter)
        if dev:
            break
        for d in await BleakScanner.discover(timeout=4, adapter=adapter):
            if d.name == "PaperPlate":
                dev = d
        if dev:
            break
    if not dev:
        print("NOT FOUND"); return
    async with BleakClient(dev, adapter=adapter) as c:
        cur = await c.read_gatt_char(CONTENT)
        print("current:", cur.decode("utf-8", "replace").split("\n"))
        if len(sys.argv) < 2:
            return
        text = "\n".join(sys.argv[1:6]).encode("utf-8")
        q = asyncio.Queue()
        await c.start_notify(STAT, lambda _h, d: q.put_nowait(struct.unpack("<BII", bytes(d[:9]))))
        ack_every = 8
        await c.write_gatt_char(CTRL, struct.pack("<BIIB", 0x11, len(text), zlib.crc32(text) & 0xFFFFFFFF, ack_every), response=True)
        st = await asyncio.wait_for(q.get(), 2)
        assert st[0] == 1, st
        mtu = getattr(c, "mtu_size", 23)
        chunk = max(16, min(mtu - 3, 125) - 2)
        off = 0
        while off < len(text):
            end = min(off + chunk * ack_every, len(text))
            while off < end:
                part = text[off:off + chunk]
                await c.write_gatt_char(DATA, struct.pack("<H", off) + part, response=False)
                off += len(part)
            st = await asyncio.wait_for(q.get(), 2)
            if st[1] < off:
                off = st[1]
        await c.write_gatt_char(CTRL, bytes([0x02]), response=False)
        st = await asyncio.wait_for(q.get(), 5)
        print("commit ->", ST.get(st[0], st[0]))
        await asyncio.sleep(1.5)
        cur = await c.read_gatt_char(CONTENT)
        print("now:", cur.decode("utf-8", "replace").split("\n"))


asyncio.run(main())
