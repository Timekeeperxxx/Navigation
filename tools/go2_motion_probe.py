#!/usr/bin/env python3
import argparse
import asyncio
import json
import time

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD, SPORT_CMD_MCF
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod


def no_reply_payload(api_id, parameter=None):
    return {
        "header": {
            "identity": {"id": int(time.time() * 1000) % 2147483648, "api_id": api_id},
            "policy": {"priority": 0, "noreply": True},
        },
        "parameter": json.dumps(parameter or {}),
        "binary": [],
    }


async def call(conn, api_id, parameter=None, label="call"):
    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    response = await conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], payload)
    code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
    data = response.get("data", {}).get("data", "")
    print(f"{label}: code={code}, data={data}")
    return code, data


async def publish_move(conn, api_id, x, y, z, duration, hz):
    period = 1.0 / hz
    count = int(duration * hz)
    for _ in range(count):
        conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["SPORT_MOD"],
            no_reply_payload(api_id, {"x": x, "y": y, "z": z}),
        )
        await asyncio.sleep(period)


async def publish_wireless(conn, lx, ly, rx, duration, hz):
    period = 1.0 / hz
    count = int(duration * hz)
    for _ in range(count):
        conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["WIRELESS_CONTROLLER"],
            {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0},
        )
        await asyncio.sleep(period)
    conn.datachannel.pub_sub.publish_without_callback(
        RTC_TOPIC["WIRELESS_CONTROLLER"],
        {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": 0},
    )


async def wait_step(label, auto):
    print(f"\n=== {label} ===")
    if not auto:
        input("Press Enter to run this test, Ctrl+C to stop...")


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["LocalAP", "LocalSTA"], default="LocalAP")
    parser.add_argument("--ip", default="192.168.8.181")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--auto", action="store_true")
    args = parser.parse_args()

    method = getattr(WebRTCConnectionMethod, args.method)
    kwargs = {"ip": args.ip} if args.method == "LocalSTA" else {}
    conn = UnitreeWebRTCConnection(method, **kwargs)
    await conn.connect()
    print("connected")

    await wait_step("StandUp then MCF Move x=0.30", args.auto)
    await call(conn, SPORT_CMD_MCF["StandUp"], label="MCF StandUp")
    await asyncio.sleep(3)
    await publish_move(conn, SPORT_CMD_MCF["Move"], 0.30, 0.0, 0.0, args.duration, args.hz)
    await call(conn, SPORT_CMD_MCF["StopMove"], label="MCF StopMove")

    await wait_step("BalanceStand then MCF Move x=0.30", args.auto)
    await call(conn, SPORT_CMD_MCF["BalanceStand"], label="MCF BalanceStand")
    await asyncio.sleep(1)
    await publish_move(conn, SPORT_CMD_MCF["Move"], 0.30, 0.0, 0.0, args.duration, args.hz)
    await call(conn, SPORT_CMD_MCF["StopMove"], label="MCF StopMove")

    await wait_step("SwitchJoystick true then wireless ly=0.90", args.auto)
    await call(conn, SPORT_CMD_MCF["SwitchJoystick"], {"data": True}, "MCF SwitchJoystick true")
    await asyncio.sleep(0.5)
    await publish_wireless(conn, 0.0, 0.90, 0.0, args.duration, args.hz)

    await wait_step("ClassicWalk true then MCF Move x=0.30", args.auto)
    await call(conn, SPORT_CMD_MCF["ClassicWalk"], {"data": True}, "MCF ClassicWalk true")
    await asyncio.sleep(0.5)
    await publish_move(conn, SPORT_CMD_MCF["Move"], 0.30, 0.0, 0.0, args.duration, args.hz)
    await call(conn, SPORT_CMD_MCF["ClassicWalk"], {"data": False}, "MCF ClassicWalk false")

    await wait_step("FreeWalk true then MCF Move x=0.30", args.auto)
    await call(conn, SPORT_CMD_MCF["FreeWalk"], {"data": True}, "MCF FreeWalk true")
    await asyncio.sleep(0.5)
    await publish_move(conn, SPORT_CMD_MCF["Move"], 0.30, 0.0, 0.0, args.duration, args.hz)
    await call(conn, SPORT_CMD_MCF["FreeWalk"], {"data": False}, "MCF FreeWalk false")

    await wait_step("Normal SPORT BalanceStand then request Move x=0.30", args.auto)
    await call(conn, SPORT_CMD["BalanceStand"], label="SPORT BalanceStand")
    await asyncio.sleep(1)
    for _ in range(int(args.duration * 5)):
        await call(conn, SPORT_CMD["Move"], {"x": 0.30, "y": 0.0, "z": 0.0}, "SPORT Move")
        await asyncio.sleep(0.2)
    await call(conn, SPORT_CMD["StopMove"], label="SPORT StopMove")

    print("\nDone")


if __name__ == "__main__":
    asyncio.run(main())
