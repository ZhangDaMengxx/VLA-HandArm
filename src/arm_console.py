#!/usr/bin/env python3
"""src/arm_console.py — NERO 7 轴臂调试控制台。JSON over stdin/stdout。

和 hand_console.py 是**同构的一对**:各自独占一条硬件通道,不依赖 ROS,
被 app_web 以子进程拉起,stdout 每行一个 JSON 帧广播给网页。

  hand_console.py  ← /dev/ttyUSB0 (RS485) ─ 手 6 关节
  arm_console.py   ← can0        (socketcan) ─ 臂 7 关节

不走 ROS 的理由和手一样:臂调试不该被 ROS 环境问题挡住。合体时两个 console
并行跑,通道不冲突(串口 vs CAN),协议同构,上层一次下发两组角度即可。

stdin 指令(每行一个 JSON):
  {"cmd":"angles","rad":[7]}    move_j 到目标角
  {"cmd":"goto_tracking_ready"}   move_j 到实时跟随准备位
  {"cmd":"speed","value":30}    速度百分比 1-100
  {"cmd":"enable"} / {"cmd":"disable"}
  {"cmd":"home"}                回零位(全 0)
  {"cmd":"estop"}               急停
  {"cmd":"reset"}               复位(解除急停冻结)
  {"cmd":"quit"}

stdout 帧:
  {"type":"ready",...}  {"type":"state",...}  {"type":"ack",...}  {"type":"error",...}

⚠ 安全:默认 --mock。真机 --no-mock 需要 can0 + 臂上电。运动指令(angles/home)
默认**被拒**,要先 {"cmd":"enable"} —— 臂是 7 自由度工业臂,不做"连上就能动"。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
from nero_arm import (NeroArm, NERO_ARM_LIMITS, NERO_HOME_POSE,
                      NERO_TRACKING_READY_POSE, ARM_JOINTS)             # noqa: E402
from stdin_lines import StdinLines                                   # noqa: E402

HOME_RAD = list(NERO_HOME_POSE)
_player = None                # ComboPlayer 实例,主循环 tick 它


def emit(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def handle(arm: NeroArm, cmd: dict, require_enable: bool,
           pending_speed: list) -> dict:
    """处理一条指令,返回 ack。限位夹取在 NeroArm.move_j 里做。

    运动类指令(angles/home)在未使能时**拒发**。臂不做"连上就能动" —— 这是和
    手页的关键差异,7 自由度工业臂扫一下的代价不一样。

    pending_speed 是单元素可变列表(用列表是为了让 handle 能改调用方的值):
    接入时臂未使能 → 初始速度没能设,挂在这里,等 enable 成功后补发。
    """
    c = cmd.get("cmd")
    moving = c in ("angles", "home", "goto_connect_pose", "goto_tracking_ready", "combo_play",
                   "tracking_begin", "tracking_angles")
    if moving and require_enable and not arm.enabled:
        return {"type": "error", "cmd": c,
                "msg": "臂未使能,运动指令被拒。先发 {\"cmd\":\"enable\"}"}
    if moving and arm.frozen:
        return {"type": "error", "cmd": c,
                "msg": "急停生效中,运动指令被拒。先发 {\"cmd\":\"reset\"} 解除"}

    if c == "angles":
        rad = cmd.get("rad") or []
        if len(rad) != 7:
            return {"type": "error", "msg": f"angles 需要 7 个值,收到 {len(rad)}"}
        ok = arm.move_j([float(x) for x in rad])
        return {"type": "ack", "cmd": c, "ok": ok,
                "target": [round(v, 4) for v in arm.target]}
    if c == "home":
        ok = arm.move_j(list(HOME_RAD))
        return {"type": "ack", "cmd": c, "ok": ok}
    if c == "goto_connect_pose":
        # 回到"接入那一刻"的位姿。断开前把臂原样交回原控制方用这个。
        # ⚠ move_j 是关节空间插值,路径不受控。比回零近,但同样可能撞东西 ——
        # 所以只在用户显式点击时执行,不放进退出流程。
        if arm.connect_pose is None:
            return {"type": "error", "cmd": c, "msg": "没有记录接入位姿"}
        ok = arm.move_j(list(arm.connect_pose))
        return {"type": "ack", "cmd": c, "ok": ok,
                "target": [round(v, 4) for v in arm.target]}
    if c == "goto_tracking_ready":
        # 摄像头真机跟随前的安全准备位。move_j 仍是关节空间插值，调用方必须
        # 先确认沿途无障碍，并等待实际关节到位后再开始锚定。
        ok = arm.move_j(list(NERO_TRACKING_READY_POSE))
        return {"type": "ack", "cmd": c, "ok": ok,
                "target": [round(v, 4) for v in arm.target]}
    if c == "speed":
        arm.set_speed_percent(int(cmd.get("value", 30)))
        return {"type": "ack", "cmd": c, "ok": True, "value": arm.speed_percent}
    if c == "enable":
        ok = arm.enable()
        out = {"type": "ack", "cmd": c, "ok": ok}
        # 接入时因未使能而挂起的初始速度,现在补发(官方时序:0x471 先于 0x151)
        if ok and pending_speed[0] is not None:
            arm.set_speed_percent(pending_speed[0])
            out["speed_percent"] = arm.speed_percent
            pending_speed[0] = None
        return out
    if c == "disable":
        return {"type": "ack", "cmd": c, "ok": arm.disable()}
    if c == "estop":
        arm.estop()
        # ⚠ 急停不是"定格" —— 关节进阻尼模式,臂**会缓慢下落**(无关节抱闸)。
        # 这条 warn 给前端弹提示用,别让人以为按下去就安全停住了。
        return {"type": "ack", "cmd": c, "ok": True, "frozen": arm.frozen,
                "enabled": arm.enabled,
                "warn": "急停已下发:全部关节进入阻尼模式。臂无抱闸,会缓慢下落,"
                        "注意下方净空。复位后需重新使能才能运动。"}
    if c == "reset":
        arm.reset()      # 内含 0x150 byte0=2 退阻尼 + 0x471 重使能
        return {"type": "ack", "cmd": c, "ok": True, "frozen": arm.frozen,
                "enabled": arm.enabled,
                "note": "已退出阻尼并重发使能(急停后 0x471 必须重发)"}

    # ---- 实时腕部跟随:latest-target CPV 位置伺服 ----
    if c == "tracking_begin":
        ok = arm.cpv_begin()
        return {"type": "ack", "cmd": c, "ok": ok,
                "msg": None if ok else arm.last_error}
    if c == "tracking_angles":
        rad = cmd.get("rad") or []
        if len(rad) != 7:
            return {"type": "error", "cmd": c,
                    "msg": f"tracking_angles 需要 7 个值,收到 {len(rad)}"}
        if not arm.cpv_active:
            return {"type": "error", "cmd": c,
                    "msg": "tracking_angles 前必须成功进入 tracking_begin/CPV"}
        ok = arm.move_cpv_pos([float(x) for x in rad])
        return {"type": "ack", "cmd": c, "ok": ok,
                "target": [round(v, 4) for v in arm.target],
                "tracking_token": cmd.get("tracking_token"),
                "frame_id": cmd.get("frame_id"),
                "msg": None if ok else arm.last_error}
    if c == "tracking_end":
        arm.cpv_end()
        return {"type": "ack", "cmd": c, "ok": True}

    # ---- 联合回放的臂侧:CPV 逐关节位置伺服 ----
    # ⚠ 为什么在 console 里跑而不在 web 层:CPV 要 `arm.move_cpv_pos()`,
    # 那是 NeroArm 的方法,而 NeroArm 只在这个进程里(它独占 can0)。
    # web 层经 stdin 递 JSON 递不了对象,所以播放器必须在这边。
    # 和 hand_console 跑 ActionPlayer 是同一个道理。
    if c == "combo_play":
        global _player
        if _player is not None and not _player.done:
            return {"type": "error", "cmd": c, "msg": "已经在回放,先发 combo_stop"}
        try:
            from combo_player import ComboPlayer, ArmTrajPack, ArmWaypoint
        except ImportError as e:
            return {"type": "error", "cmd": c, "msg": f"combo_player 不可用: {e}"}
        wps = cmd.get("waypoints") or []
        if not wps:
            return {"type": "error", "cmd": c, "msg": "waypoints 为空"}
        try:
            pts = [ArmWaypoint(t_ns=int(w["t_ns"]), rad=[float(v) for v in w["rad"]])
                   for w in wps]
        except (KeyError, TypeError, ValueError) as e:
            return {"type": "error", "cmd": c, "msg": f"waypoints 不合法: {e}"}
        for i, w in enumerate(pts):
            if len(w.rad) != 7:
                return {"type": "error", "cmd": c,
                        "msg": f"waypoint[{i}] 需要 7 个角,收到 {len(w.rad)}"}
        # ⚠ 只查不夹 —— 夹了就把"包本身越界"这个事实抹掉了(和 combo_player
        # 的入口校验同一条纪律)。1e-4 容差挡取整残渣。
        for i, w in enumerate(pts):
            for j, v in enumerate(w.rad):
                lo, hi = NERO_ARM_LIMITS[j]
                if v < lo - 1e-4 or v > hi + 1e-4:
                    return {"type": "error", "cmd": c,
                            "msg": f"waypoint[{i}] {ARM_JOINTS[j]}={v:.4f} 超限位"}
        pack = ArmTrajPack(name=str(cmd.get("name") or "联合回放"),
                           mode=str(cmd.get("mode") or "waypoints"),
                           waypoints=pts, approach_rad=list(pts[0].rad))
        # hand=None:手侧由 hand_console 自己播(两个进程各持一个设备)。
        # 两侧靠 start_at(共同的 CLOCK_MONOTONIC 时刻)对齐 —— 见 main() 里
        # 的 tick 和 web 层 _combo_start 的注释。
        _player = ComboPlayer(pack, arm, None, None,
                              skip_arm=(pack.mode == "stream"))
        bad = _player.preflight()
        if bad:
            _player = None
            return {"type": "error", "cmd": c, "msg": "preflight 不通过: " + "; ".join(bad)}
        # approach:慢速挪到首帧,挡住过大的初始落差。
        ok_ap, msg_ap = _player.approach(speed_pct=10)
        if not ok_ap:
            _player = None
            return {"type": "error", "cmd": c, "msg": f"approach 失败: {msg_ap}"}
        # 进 CPV 模式 —— 关掉 SDK 的 auto_set_motion_mode,后续逐关节位置指令才有效。
        if not arm.mock and not arm.cpv_begin():
            _player = None
            return {"type": "error", "cmd": c, "msg": f"进入 CPV 失败: {arm.last_error}"}
        start_at = float(cmd.get("start_at") or 0.0) or None
        _player.start(start_at=start_at)
        return {"type": "ack", "cmd": c, "ok": True, "name": pack.name,
                "waypoints": len(pts), "mode": pack.mode,
                "duration_s": round(pack.dur_ns / 1e9, 2), "cpv": True}
    if c in ("combo_pause", "combo_resume", "combo_stop"):
        if _player is None:
            return {"type": "error", "cmd": c, "msg": "没有在回放"}
        if c == "combo_pause":
            _player.pause()
        elif c == "combo_resume":
            _player.resume()
        else:
            _player.stop()
            # ⚠ stop 之后**不清 _player** —— 主循环要靠它走到 cpv_end()
            # 把 auto_set_motion_mode 恢复。清早了那一步就没人做,
            # 之后的 move_j 会走在 cpv 模式下(行为未定义)。
        return {"type": "ack", "cmd": c, "ok": True, "paused": _player.paused}
    return {"type": "error", "msg": f"未知指令: {c}"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # ⚠ 默认 mock,和 hand_console 相反。臂的伤害量级不同,不做"默认接真机"。
    ap.add_argument("--mock", dest="mock", action="store_true", default=True,
                    help="无硬件空跑(默认)")
    ap.add_argument("--no-mock", dest="mock", action="store_false",
                    help="真机 CAN(需 can0 + 臂上电)")
    ap.add_argument("--channel", default="can0")
    # auto = 先用 DEFAULT driver 读 software_version 再按 SDK 门限重连(NeroArm._detect_firmware)。
    # 默认改成 auto:这台臂实测 1.11 → v111,原来的 "default" 是**我们填的猜测值**,
    # 不是探到的。写死 v111 也不行 —— 固件升级后会静默错。多几百毫秒换掉一个静默错误。
    ap.add_argument("--firmware", default="auto",
                    choices=["auto", "default", "v111", "v112", "v120"])
    ap.add_argument("--hz", type=float, default=20.0, help="遥测发布率")
    ap.add_argument("--speed", type=int, default=20,
                    help="初始速度百分比 1-100。默认压到 20 —— 调试期宁慢勿快")
    ap.add_argument("--full-telemetry-every", type=float, default=1.0,
                    help="全量遥测(电流/力矩/状态)间隔秒;关节角仍按 --hz 读")
    ap.add_argument("--allow-motion-without-enable", action="store_true",
                    help="允许未使能就发运动指令(默认关,不建议开)")
    args = ap.parse_args()

    arm = NeroArm(mock=args.mock, channel=args.channel, firmware=args.firmware)
    try:
        arm.connect()
    except Exception as e:                                   # noqa: BLE001
        emit({"type": "error", "fatal": True, "msg": str(e)})
        return
    # ⚠ 官方流程是 0x471 使能 **在** 0x151 之前:
    #   2.发送0x471指令使能全部关节电机  →  3.发送0x151指令进入CAN控制模式
    # set_speed_percent() 发的正是 0x151。臂已使能时(接入松灵客户端的常态)顺序无碍,
    # 但从待机冷启动时抢先发 0x151 不符合官方时序。所以:已使能才现在设速度,
    # 未使能则挂起,等 {"cmd":"enable"} 之后补 —— 见 handle() 里的 pending_speed。
    pending_speed = [None]
    if arm.enabled:
        arm.set_speed_percent(args.speed)
    else:
        pending_speed[0] = args.speed
    emit({"type": "ready", "mock": args.mock, "channel": args.channel,
          # firmware 报**实际生效**的 driver,不是命令行里那个 "auto"。
          # 页面上看到 "auto" 等于什么都没说。
          "firmware": arm.firmware_detected or args.firmware,
          # ⚠ speed_percent 是**当前生效值**;未使能时 --speed 挂起没发下去,
          # 这时两者不一样。只报前者会让页面把 100 当成已生效 —— 所以把挂起值也报出来,
          # 让"看到的数"和"臂里的数"能对上。生效后 pending_speed 恒为 null。
          "pending_speed": pending_speed[0],
          "joints": ARM_JOINTS,
          "limits": [list(t) for t in NERO_ARM_LIMITS],
          "speed_percent": arm.speed_percent,
          # 接入位姿 + **真实**使能状态(不是本地默认值),前端据此决定按钮可用性
          "connect_pose": ([round(v, 4) for v in arm.connect_pose]
                           if arm.connect_pose else None),
          "enabled": arm.enabled,
          "require_enable": not args.allow_motion_without_enable})

    dt = 1.0 / max(1.0, args.hz)
    t0 = time.monotonic()
    last_full = 0.0
    next_tick = time.monotonic()
    require_enable = not args.allow_motion_without_enable
    stdin_lines = StdinLines()
    try:
        while True:
            # 阻塞等到下一个 tick —— 命令一到立刻醒。
            # 和 hand_console 同一套时序,别改回 timeout=0 + 无条件 sleep。
            # ⚠ 也别改回 select + sys.stdin.readline:select 看 fd、readline 看
            #   用户态缓冲,一次写进多行时会有行卡在缓冲里 = 慢一个命令。
            #   见 stdin_lines.py 的模块说明。
            timeout = max(0.0, next_tick - time.monotonic())
            for line in stdin_lines.poll(timeout):
                try:
                    cmd = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if cmd.get("cmd") == "quit":
                    raise KeyboardInterrupt
                emit(handle(arm, cmd, require_enable, pending_speed))
            if stdin_lines.eof:                          # stdin 关闭 = 上层退出
                raise KeyboardInterrupt

            now = time.monotonic()
            # ⚠ 挂起的速度要在**观察到已使能**时补,不能只在我们自己发 enable 时补。
            # 臂常态是松灵客户端控制着,可能被**外部**使能 —— 那条路径不经过 handle(),
            # 于是 --speed 20 永远发不下去,臂按上一次(未知,可能是 100%)的速度动。
            # 实测踩到:真机 --speed 20 起的,遥测里却是 100 而且一直没被纠正。
            if pending_speed[0] is not None and arm.enabled:
                arm.set_speed_percent(pending_speed[0])
                emit({"type": "note", "msg": f"已使能,补发速度 {pending_speed[0]}%"})
                pending_speed[0] = None

            # ⚠ 回放器 tick:必须在读位姿**之前**,否则 CPV 帧在位姿读之后才发,
            # 遥测里的 target 滞后一拍。
            global _player
            if _player is not None:
                if not _player.done:
                    _player.tick()
                else:
                    # 跑完了:恢复 auto_set_motion_mode,清 _player。
                    # ⚠ cpv_end 要在这里调而不是在 combo_stop 时 —— stop 是「中断」,
                    # 那时用户可能接着发下一个包,两个包之间不该退出 CPV 模式(退了
                    # 下一个 combo_play 又要重进,多一次模式切换)。只在 done 时退,
                    # 意思是「一段时间内不会再有 CPV」了。
                    if not arm.mock:
                        arm.cpv_end()
                    emit({"type": "combo_done", "name": _player.pack.name,
                          "stopped": _player.stopped,
                          "sent": _player.sent_arm, "fail": _player.fail_arm})
                    _player = None

            rad = arm.read_angles()
            row = {"type": "state", "t": round(now - t0, 3),
                   "names": ARM_JOINTS, "rad": [round(v, 4) for v in rad],
                   "target": [round(v, 4) for v in arm.target],
                   "enabled": arm.enabled, "frozen": arm.frozen,
                   "speed_percent": arm.speed_percent,
                   # 与接入位姿的最大单关节偏差(rad)。前端用它决定断开时是否弹确认。
                   "pose_drift": (None if arm.connect_pose is None else
                                  round(max(abs(a - b) for a, b in
                                            zip(rad, arm.connect_pose)), 4))}
            # 回放进度随遥测捎带 —— 前端不用另开一路轮询。
            if _player is not None:
                row["combo"] = {"name": _player.pack.name,
                                "progress": round(_player.progress(), 4),
                                # max(0,…):start_at 在未来时 elapsed 是负的
                                "elapsed_ms": max(0, _player.elapsed_ns // 1_000_000),
                                "total_ms": _player.total_ns // 1_000_000,
                                "paused": _player.paused,
                                "i": _player.i_arm,
                                "n": len(_player.pack.waypoints),
                                "fail": _player.fail_arm}
            if now - last_full >= args.full_telemetry_every:
                last_full = now
                row["tel"] = arm.telemetry()
                if arm.last_error:
                    row["last_error"] = arm.last_error
            emit(row)
            next_tick += dt
            if next_tick < now:                          # 落后太多就重新对齐
                next_tick = now + dt
    except KeyboardInterrupt:
        pass
    finally:
        # 退出时**什么都不改**,只断 CAN。原样把臂交回给原来的控制方(常态是松灵客户端)。
        #
        # 不回零:臂在半途,回零路径未知,可能撞到工装/桌面/自己。想回位姿走
        # goto_connect_pose,那是你主动点、看着它走的动作,不是退出时偷偷跑。
        #
        # 不去使能:实测失能仍靠伺服锁死位置(不掉扭矩),所以 disable() 既救不了
        # "垂下来"、又白改一次臂的状态,还让松灵那边接手前要补一次使能。
        # 详见 ARM_DEBUG.md「使能 ≠ 掉扭矩」。
        arm.disconnect()
        emit({"type": "closed"})


if __name__ == "__main__":
    main()
