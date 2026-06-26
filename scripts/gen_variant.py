"""Post-build: emit .pio/build/<env>/variant.json (role/transport/board/label).

Studio's firmware library groups builds by node role (DEC-034,
contracts/specs/firmware-distribution.md). Without this file the dev plugin
falls back to env-name inference, which can't classify names like "esp32"
and would misfile the Bridge relay under the Hapbeat (receiver) group.
"""
import json
import os

Import("env")  # noqa: F821  (PlatformIO construction environment)

# Label format: "<device> (<purpose>)" — matches the receiver-side format
# used by hapbeat-device-firmware/merge_firmware.py so the Studio firmware
# library reads as "device name first" across all node types (was
# previously "<purpose> (<device>)" — inverted relative to receivers).
VARIANTS = {
    # Bridge command relay: PC (Bridge) -> USB serial -> ESP-NOW commands.
    # board names the module (M5Stack Basic, classic ESP32), not the MCU.
    "esp32": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_basic",
        "label": "M5Stack Basic (Bridge 中継機)",
        "description": "Bridge からのコマンドを ESP-NOW で Hapbeat 群に同報する中継機。",
    },
    # Live audio source: PA line-in -> ADPCM -> ESP-NOW 0xAA stream (DEC-034/DEC-033).
    # Includes piggyback (§3.2) and in-flight control for robust low-latency streaming.
    "m5stack_audio_tx": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_basic",
        "label": "M5Stack Basic (ライブ送信機)",
        "description": "PA ライン入力を ESP-NOW で同報するライブ音声送信機。",
    },
    # Repeater: re-broadcasts 0xAA packets from a configured source MAC (DEC-033).
    # 1-hop relay; loop-prevention via single-source-MAC allowlist.
    "m5stack_repeater": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_basic",
        "label": "M5Stack Basic (リピータ)",
        "description": "音声ソース機からの ESP-NOW ストリームを中継して受信機のカバレッジを拡張する。",
    },
}


def write_variant(source, target, env):  # noqa: ARG001
    pioenv = env.subst("$PIOENV")
    meta = VARIANTS.get(pioenv)
    if meta is None:
        return
    build_dir = env.subst("$BUILD_DIR")
    out = os.path.join(build_dir, "variant.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[variant] Wrote variant.json: role={meta['role']} label={meta['label']}")

    # Stable dist/ copy — `.pio/build` is volatile (cleans / parallel
    # sessions prune other envs), which made the Studio dev firmware
    # library fall back to its snapshot cache. dist/<env>/ (gitignored)
    # survives and is what the Studio dev plugin reads first. Mirrors
    # hapbeat-device-firmware/merge_firmware.py.
    import shutil
    dist_dir = os.path.join(env.subst("$PROJECT_DIR"), "dist", pioenv)
    os.makedirs(dist_dir, exist_ok=True)
    for name in ("firmware.bin", "variant.json"):
        src = os.path.join(build_dir, name)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(dist_dir, name))
    print(f"[variant] Copied distributables to dist/{pioenv}/")


env.AddPostAction("$BUILD_DIR/firmware.bin", write_variant)  # noqa: F821
