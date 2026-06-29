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
    # Live audio source on CoreS3 (ESP32-S3) + Module Audio (ES8388), via
    # M5Unified + M5Module-Audio. Same 0xAA stream as m5stack_audio_tx.
    "m5stack_cores3_audio_tx": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_cores3",
        "label": "M5Stack CoreS3 (ライブ送信機)",
        "description": "CoreS3 + Module Audio(ES8388) で PA ライン入力を ESP-NOW 同報するライブ音声送信機。",
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


def _read_build_version(project_dir):
    """Return (fwVersion, buildCommit) from the generated src/build_version.h
    (written by scripts/build_version.py pre-build, DEC-035). ('', '') if absent."""
    import re
    path = os.path.join(project_dir, "src", "build_version.h")
    fw, commit = "", ""
    if os.path.isfile(path):
        try:
            with open(path, encoding="utf-8") as f:
                text = f.read()
            m = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', text)
            if m:
                fw = m.group(1)
            mc = re.search(r'#define\s+BUILD_COMMIT_SHA\s+"([^"]+)"', text)
            if mc:
                commit = mc.group(1)
        except OSError:
            pass
    return fw, commit


def write_variant(source, target, env):  # noqa: ARG001
    pioenv = env.subst("$PIOENV")
    base = VARIANTS.get(pioenv)
    if base is None:
        return
    # Copy so per-env version metadata doesn't accumulate on the shared dict.
    meta = dict(base)
    fw, commit = _read_build_version(env.subst("$PROJECT_DIR"))
    if fw:
        meta["fwVersion"] = fw          # per-env semver (DEC-035)
    if commit:
        meta["buildCommit"] = commit    # audit
    build_dir = env.subst("$BUILD_DIR")
    out = os.path.join(build_dir, "variant.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[variant] Wrote variant.json: role={meta['role']} "
          f"label={meta['label']} fwVersion={fw or '(none)'}")

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
