#!/usr/bin/env python3
"""Host-side BLE checks of the OpenTread head unit (BLE name `OpenTread`): advertising,
GATT reads, notifications and the FTMS Control Point. Uses bleak (the repo's .venv).

    .venv/bin/python tools/ble_check.py scan               advert: name, UUIDs, FTMS Service Data
    .venv/bin/python tools/ble_check.py read               RSC / DIS / FTMS reads, checked
    .venv/bin/python tools/ble_check.py subscribe --rsc --ftms-sub --seconds 30
                                                             notifications, decoded (compare with R/X lines)
    .venv/bin/python tools/ble_check.py cp                 FTMS Control Point request/response matrix
    .venv/bin/python tools/ble_check.py all                scan + read + 10 s subscribe + cp

Options: --name OpenTread (default), --address <CoreBluetooth UUID> (skips the
name scan), --timeout 10 (scan seconds).

Control Point matrix rule: run it from a SECOND central while a
dummy central (e.g. nRF Connect on the phone) is connected first and holds
connection handle 0, so the zeroed-connInfo bug class of NimBLE-Arduino 2.5.0
can't hide behind handle 0. The script asks for confirmation before it starts.

Every check prints PASS/FAIL; the exit code is 1 if any check failed.
macOS caveat: CoreBluetooth merges the advertisement and the scan response and
doesn't expose the raw bytes, so `scan` checks the merged content (name,
service UUIDs, FTMS Service Data), not the 23/28/31-byte layouts (those are
covered by the firmware's host tests and printed by the firmware's boot line
`ble;adv layout=... primary=<hex> scan_rsp=<hex>`).
"""
from __future__ import annotations

import argparse
import asyncio
import re
import sys
import time

from bleak import BleakClient, BleakScanner


def u(x: int) -> str:
    return f"0000{x:04x}-0000-1000-8000-00805f9b34fb"


RSC_SERVICE, RSC_MEAS, RSC_FEATURE, SENSOR_LOC = u(0x1814), u(0x2A53), u(0x2A54), u(0x2A5D)
DIS_SERVICE, DIS_MODEL, DIS_FW, DIS_HW, DIS_MANUF = u(0x180A), u(0x2A24), u(0x2A26), u(0x2A27), u(0x2A29)
FTMS_SERVICE, FTMS_FEATURE, TREADMILL_DATA = u(0x1826), u(0x2ACC), u(0x2ACD)
TRAINING_STATUS, CONTROL_POINT, MACHINE_STATUS = u(0x2AD3), u(0x2AD9), u(0x2ADA)

FAILS: list[str] = []
DISCONNECT_WAIT_S = 10.0  # cp: wait for the first session's disconnected_callback
NEW_LINK_GAP_S = 3.0      # cp: then this long before the "new connection"


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)
    return ok


def decode_rsc(b: bytes) -> str:
    if len(b) != 8:
        return f"len={len(b)} (expected 8)"
    flags, sp, cad, dm = b[0], int.from_bytes(b[1:3], "little"), b[3], int.from_bytes(b[4:8], "little")
    return (f"flags={flags:02x}{' running' if flags & 0x04 else ''} speed={sp}/256 m/s = {sp / 256 * 3.6:.2f} km/h "
            f"cadence={cad} distance={dm} dm = {dm / 10:.1f} m")


def decode_td(b: bytes) -> str:
    if len(b) != 9:
        return f"len={len(b)} (expected 9)"
    flags = int.from_bytes(b[0:2], "little")
    sp, dist, el = int.from_bytes(b[2:4], "little"), int.from_bytes(b[4:7], "little"), int.from_bytes(b[7:9], "little")
    return f"flags={flags:04x} speed={sp / 100:.2f} km/h distance={dist} m elapsed={el} s"


async def find(args):
    if args.address:
        return args.address
    print(f"scanning {args.timeout:.0f} s for '{args.name}' ...")
    dev = await BleakScanner.find_device_by_name(args.name, timeout=args.timeout)
    if dev is None:
        check("discover", False, f"no '{args.name}' advert (3 links already up?)")
        return None
    return dev


async def cmd_scan(args):
    print("== scan")
    found = {}

    def cb(dev, adv):
        if adv.local_name == args.name:
            found[dev.address] = (dev, adv)

    async with BleakScanner(cb):
        await asyncio.sleep(args.timeout)
    if not check("discover", bool(found), f"{len(found)} device(s) named {args.name}"):
        return
    for addr, (dev, adv) in found.items():
        uuids = [x.lower() for x in adv.service_uuids]
        print(f"  {addr} rssi={adv.rssi} name={adv.local_name} uuids={uuids} service_data="
              f"{ {k: v.hex() for k, v in adv.service_data.items()} }")
        check("name in the advert", adv.local_name == args.name)
        check("RSC 0x1814 advertised", RSC_SERVICE in uuids)
        sd = adv.service_data.get(FTMS_SERVICE)
        if args.ftms == "1":
            check("FTMS advertised (UUID or Service Data)", FTMS_SERVICE in uuids or sd is not None)
            if sd is not None:
                check("FTMS Service Data = 01 01 00 (available, treadmill)", sd == bytes([0x01, 0x01, 0x00]), sd.hex())
        else:
            check("no FTMS in an ftms=0 build", FTMS_SERVICE not in uuids and sd is None)


async def cmd_read(args, client=None):
    print("== read")
    own = client is None
    if own:
        dev = await find(args)
        if dev is None:
            return
        client = BleakClient(dev)
        await client.connect()
    try:
        r = lambda c: client.read_gatt_char(c)  # noqa: E731
        check("RSC Feature = 06 00", (await r(RSC_FEATURE)) == bytes([0x06, 0x00]))
        check("Sensor Location = 00", (await r(SENSOR_LOC)) == bytes([0x00]))
        manuf = (await r(DIS_MANUF)).decode(errors="replace")
        model = (await r(DIS_MODEL)).decode(errors="replace")
        fw = (await r(DIS_FW)).decode(errors="replace")
        hw = (await r(DIS_HW)).decode(errors="replace")
        print(f"  DIS manufacturer={manuf!r} model={model!r} firmware={fw!r} hardware={hw!r}")
        check("DIS never the stock identity", "AssaultRunner" not in manuf + model)
        check("DIS manufacturer OpenTread / model OpenTread head unit", manuf == "OpenTread" and model == "OpenTread head unit")
        check("DIS firmware <x.y.z>+<id8>", re.match(r"\d+\.\d+\.\d+\+[0-9a-f]{8}", fw) is not None, fw)
        svcs = {s.uuid.lower() for s in client.services}
        if args.ftms == "1":
            if check("FTMS service present", FTMS_SERVICE in svcs):
                check("FTMS Feature = 04 10 00 00 00 00 00 00",
                      (await r(FTMS_FEATURE)) == bytes([0x04, 0x10, 0, 0, 0, 0, 0, 0]))
                ts = await r(TRAINING_STATUS)
                check("Training Status readable [00, code]", len(ts) == 2 and ts[0] == 0 and ts[1] in (0x01, 0x0D, 0x0F),
                      ts.hex())
                chars = {c.uuid.lower(): c for s in client.services for c in s.characteristics}
                td = chars.get(TREADMILL_DATA)
                check("Treadmill Data NOTIFY only (no READ)", td is not None and "notify" in td.properties
                      and "read" not in td.properties, str(td.properties if td else None))
                cp = chars.get(CONTROL_POINT)
                check("Control Point WRITE + WRITE_NR + INDICATE",
                      cp is not None and {"write", "write-without-response", "indicate"} <= set(cp.properties),
                      str(cp.properties if cp else None))
                meas = chars.get(RSC_MEAS)
                check("RSC Measurement NOTIFY only", meas is not None and meas.properties == ["notify"],
                      str(meas.properties if meas else None))
        else:
            check("no FTMS service in an ftms=0 build", FTMS_SERVICE not in svcs)
    finally:
        if own:
            await client.disconnect()


async def cmd_subscribe(args, client=None):
    print(f"== subscribe ({'RSC ' if args.rsc else ''}{'FTMS' if args.ftms_sub else ''}) for {args.seconds:.0f} s")
    own = client is None
    if own:
        dev = await find(args)
        if dev is None:
            return
        client = BleakClient(dev)
        await client.connect()
    counts = {"rsc": 0, "td": 0, "ts": 0, "ms": 0}
    t0 = time.monotonic()
    last_dm = [-1]

    def on_rsc(_, data: bytearray):
        counts["rsc"] += 1
        b = bytes(data)
        dm = int.from_bytes(b[4:8], "little") if len(b) == 8 else -1
        if dm < last_dm[0]:
            check("RSC distance never decreases", False, f"{last_dm[0]} -> {dm}")
        last_dm[0] = max(last_dm[0], dm)
        print(f"  {time.monotonic() - t0:7.2f} s  R {b.hex()}  {decode_rsc(b)}")

    def on_td(_, data: bytearray):
        counts["td"] += 1
        print(f"  {time.monotonic() - t0:7.2f} s  X {bytes(data).hex()}  {decode_td(bytes(data))}")

    def on_ts(_, data: bytearray):
        counts["ts"] += 1
        print(f"  {time.monotonic() - t0:7.2f} s  Training Status {bytes(data).hex()}")

    def on_ms(_, data: bytearray):
        counts["ms"] += 1
        print(f"  {time.monotonic() - t0:7.2f} s  Machine Status {bytes(data).hex()}")

    try:
        if args.rsc:
            await client.start_notify(RSC_MEAS, on_rsc)
        if args.ftms_sub:
            await client.start_notify(TREADMILL_DATA, on_td)
            await client.start_notify(TRAINING_STATUS, on_ts)
            await client.start_notify(MACHINE_STATUS, on_ms)
        await asyncio.sleep(args.seconds)
    finally:
        if own:
            await client.disconnect()
    n = int(args.seconds)
    if args.rsc:
        check("RSC ~1 Hz", n - 2 <= counts["rsc"] <= n + 1, f"{counts['rsc']} in {n} s")
    if args.ftms_sub:
        check("Treadmill Data ~1 Hz", n - 2 <= counts["td"] <= n + 1, f"{counts['td']} in {n} s")
    print("  compare the payloads above with the R / X lines of the serial log (byte-identical)")


async def cmd_cp(args):
    print("== Control Point matrix")
    if not args.yes:
        input("  Connect the DUMMY central first (nRF Connect on the phone: connect to OpenTread and stay\n"
              "  connected), then press Enter ... ")
    dev = await find(args)
    if dev is None:
        return
    q: asyncio.Queue = asyncio.Queue()

    def on_ind(_, data: bytearray):
        q.put_nowait(bytes(data))

    async def step(client, req: bytes, want, label, response=True):
        while not q.empty():
            q.get_nowait()
        await client.write_gatt_char(CONTROL_POINT, req, response=response)
        try:
            got = await asyncio.wait_for(q.get(), timeout=2.0)
        except asyncio.TimeoutError:
            got = None
        check(f"{label}: write {req.hex()} -> {want.hex() if want else 'nothing'}", got == want,
              f"got {got.hex() if got else 'nothing'}")

    # The "new connection" step must really be a NEW link.
    # Wait for bleak's disconnected_callback of the first session, then
    # NEW_LINK_GAP_S more, so CoreBluetooth has torn the link down (and the
    # firmware has logged B disconnect + restarted advertising) before the
    # second connect; otherwise macOS may reuse the old link and the "grant
    # must not survive" check proves nothing.
    gone = asyncio.Event()
    loop = asyncio.get_running_loop()

    def on_disconnect(_client):
        loop.call_soon_threadsafe(gone.set)

    async with BleakClient(dev, disconnected_callback=on_disconnect) as c:
        await step(c, bytes([0x00]), None, "no indications enabled (firmware logs B cp_drop no_cccd)")
        await c.start_notify(CONTROL_POINT, on_ind)
        await step(c, bytes([0x00]), bytes([0x80, 0x00, 0x01]), "indications on (B cp_ind then cp_ack)")
    try:
        await asyncio.wait_for(gone.wait(), timeout=DISCONNECT_WAIT_S)
        check("first session disconnected (bleak disconnected_callback)", True)
    except asyncio.TimeoutError:
        check("first session disconnected (bleak disconnected_callback)", False,
              f"no callback within {DISCONNECT_WAIT_S:.0f} s")
    await asyncio.sleep(NEW_LINK_GAP_S)
    print("  OPERATOR: in the serial log, confirm a 'B,<t>,disconnect,<conn>,reason=...' line between the first\n"
          "  session's cp_ack and the second session's 'B,<t>,connect,...' line (a real new link), else this\n"
          "  'new connection' step is not valid evidence")
    async with BleakClient(dev) as c:       # "a new connection": the grant must not survive
        await c.start_notify(CONTROL_POINT, on_ind)
        await step(c, bytes([0x07]), bytes([0x80, 0x07, 0x05]), "new connection, 07 before 00")
        await step(c, bytes([0x02, 0x64, 0x00]), bytes([0x80, 0x02, 0x02]), "target speed (not supported)")
        await step(c, bytes([0x08, 0x03]), bytes([0x80, 0x08, 0x03]), "stop/pause with a bad parameter")
        await step(c, bytes([0x00]), bytes([0x80, 0x00, 0x01]), "request control")
        await step(c, bytes([0x07]), bytes([0x80, 0x07, 0x01]), "start/resume (token released)")
        await step(c, bytes([0x08, 0x01]), bytes([0x80, 0x08, 0x01]), "stop")
        await step(c, bytes([0x07]), bytes([0x80, 0x07, 0x01]), "start/resume via WRITE_NR", response=False)
        await step(c, bytes([0x80]), bytes([0x80, 0x80, 0x02]), "op 0x80 (not supported)")
    print("  then check the serial log: every cp_ind has its cp_ack, no cp_timeout, no stuck token;"
          " tools/replay.py ble_cp PASS")


async def main_async(args):
    if args.cmd in ("scan", "all"):
        await cmd_scan(args)
    if args.cmd in ("read", "all"):
        await cmd_read(args)
    if args.cmd == "subscribe":
        await cmd_subscribe(args)
    if args.cmd == "all":
        args.rsc, args.ftms_sub, args.seconds = True, args.ftms == "1", 10.0
        await cmd_subscribe(args)
    if args.cmd in ("cp", "all") and args.ftms == "1":
        await cmd_cp(args)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="BLE checks of the OpenTread head unit.")
    ap.add_argument("cmd", choices=("scan", "read", "subscribe", "cp", "all"))
    ap.add_argument("--name", default="OpenTread")
    ap.add_argument("--address", help="CoreBluetooth UUID (skip the name scan)")
    ap.add_argument("--timeout", type=float, default=10.0, help="scan seconds")
    ap.add_argument("--ftms", choices=("0", "1"), default="1", help="the build's banner ftms= (default 1)")
    ap.add_argument("--rsc", action="store_true", help="subscribe: RSC Measurement")
    ap.add_argument("--ftms-sub", "--ftms-data", dest="ftms_sub", action="store_true",
                    help="subscribe: Treadmill Data + Training/Machine Status")
    ap.add_argument("--seconds", type=float, default=30.0, help="subscribe duration")
    ap.add_argument("--yes", action="store_true", help="cp: don't wait for the dummy-central confirmation")
    args = ap.parse_args(argv)
    if args.cmd == "subscribe" and not (args.rsc or args.ftms_sub):
        ap.error("subscribe needs --rsc and/or --ftms-sub")
    asyncio.run(main_async(args))
    print(f"ble_check: {'PASS' if not FAILS else 'FAIL (' + ', '.join(FAILS) + ')'}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
