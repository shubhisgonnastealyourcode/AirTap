# ============================================================
# WristGesture v1.2 BLE Monitor
# Python 3.10+
#
# Install:
#   pip install bleak
#
# Run:
#   python wrist_monitor.py
#
# ============================================================

import asyncio
import json
from bleak import BleakScanner, BleakClient

# ============================================================
# BLE CONFIG
# ============================================================

DEVICE_NAME = "WristGesture"

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# ============================================================
# NOTIFICATION CALLBACK
# ============================================================

def notification_handler(sender, data):

    try:
        text = data.decode("utf-8").strip()

        packet = json.loads(text)

        packet_type = packet.get("t")

        # ----------------------------------------------------
        # LIVE DATA PACKET
        # ----------------------------------------------------
        if packet_type == "d":

            orientation = packet.get("o")

            ax = packet.get("ax")
            ay = packet.get("ay")
            az = packet.get("az")

            gx = packet.get("gx")
            gy = packet.get("gy")
            gz = packet.get("gz")

            jerk = packet.get("j")

            print(
                f"\r"
                f"O:{orientation:<6} | "
                f"A[{ax:+.2f} {ay:+.2f} {az:+.2f}] | "
                f"G[{gx:+6.1f} {gy:+6.1f} {gz:+6.1f}] | "
                f"J:{jerk:6.2f}",
                end=""
            )

        # ----------------------------------------------------
        # GESTURE EVENT
        # ----------------------------------------------------
        elif packet_type == "p":

            gesture_num = packet.get("n")

            if gesture_num == 1:
                print("\n\n🔥 SINGLE PINCH DETECTED\n")

            elif gesture_num == 2:
                print("\n\n⚡ DOUBLE PINCH DETECTED\n")

            else:
                print(f"\n\n[UNKNOWN GESTURE] {gesture_num}\n")

        else:
            print("\n[UNKNOWN PACKET]", packet)

    except UnicodeDecodeError:
        print("\n[ERROR] Non-UTF8 packet received")

    except json.JSONDecodeError:
        print("\n[ERROR] Invalid JSON packet")

    except Exception as e:
        print("\n[ERROR]", e)

# ============================================================
# MAIN
# ============================================================

async def main():

    print("=" * 60)
    print(" WristGesture v1.2 BLE Monitor")
    print("=" * 60)

    print(f"\nScanning for '{DEVICE_NAME}'...\n")

    device = None

    devices = await BleakScanner.discover(timeout=8.0)

    for d in devices:

        name = d.name if d.name else "<unnamed>"

        print(f"{name:25} {d.address}")

        if d.name == DEVICE_NAME:
            device = d

    if not device:
        print("\n[ERROR] WristGesture not found")
        return

    print(f"\nFOUND: {device.address}")

    async with BleakClient(device.address) as client:

        if not client.is_connected:
            print("[ERROR] Failed to connect")
            return

        print("CONNECTED")

        # Start BLE notifications
        await client.start_notify(
            CHAR_UUID,
            notification_handler
        )

        print("\nListening for live telemetry...\n")
        print("Press CTRL+C to quit.\n")

        while True:
            await asyncio.sleep(1)

# ============================================================
# ENTRY
# ============================================================

try:
    asyncio.run(main())

except KeyboardInterrupt:
    print("\n\nDisconnected.")