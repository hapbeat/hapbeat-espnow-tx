"""Post-build: emit the Studio-flashable artifacts for a transmitter env.

  firmware.bin              app-only (PlatformIO original; kept for internal use)
  firmware_app_ota.bin      app-only, tagged (Wi-Fi OTA — unused on TX but emitted
                            for symmetry with the receiver distribution)
  firmware_full_serial.bin  merged bootloader + partitions(0x8000) + app(0x10000)
                            — this is what Studio flashes over USB serial
  variant.json              role/transport/board/label (+ content hashes)

Studio requires a MERGED firmware_full_serial.bin (bootloader + partitions +
app); without it the flasher rejects the build ("not a merged image"). This
mirrors hapbeat-device-firmware/merge_firmware.py, minus the receiver-specific
variant.json inference (the transmitter uses the explicit VARIANTS table below).

`pio run -t upload` is overridden to write firmware_full_serial.bin via an
NVS-safe split (skip 0x9000-0xFFFF so channel / mode / input_level survive;
reset otadata so the freshly-flashed app boots).
"""
import json
import os

Import("env")  # noqa: F821  (PlatformIO construction environment)

# Label format: "<device> (<purpose>)" — matches the receiver-side format
# used by hapbeat-device-firmware/merge_firmware.py so the Studio firmware
# library reads as "device name first" across all node types.
VARIANTS = {
    "esp32": {
        "role": "transmitter", "transport": "espnow_stream", "board": "m5stack_basic",
        "label": "M5Stack Basic (Bridge 中継機)",
        "description": "Bridge からのコマンドを ESP-NOW で Hapbeat 群に同報する中継機。",
    },
    "m5stack_audio_tx": {
        "role": "transmitter", "transport": "espnow_stream", "board": "m5stack_basic",
        "label": "M5Stack Basic (ライブ送信機)",
        "description": "PA ライン入力を ESP-NOW で同報するライブ音声送信機。",
    },
    "m5stack_cores3_audio_tx": {
        "role": "transmitter", "transport": "espnow_stream", "board": "m5stack_cores3",
        "label": "M5Stack CoreS3 (ライブ送信機)",
        "description": "CoreS3 + Module Audio(ES8388) で PA ライン入力を ESP-NOW 同報するライブ音声送信機。",
    },
    "m5stack_repeater": {
        "role": "transmitter", "transport": "espnow_stream", "board": "m5stack_basic",
        "label": "M5Stack Basic (リピータ)",
        "description": "音声ソース機からの ESP-NOW ストリームを中継して受信機のカバレッジを拡張する。",
    },
}


def _read_build_version(project_dir):
    """Return (fwVersion, buildCommit) from the generated src/build_version.h."""
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


def _sha256(path):
    import hashlib
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()


def _merge_serial(env, build_dir, fw_version):
    """Produce firmware_app_ota.bin + firmware_full_serial.bin (merged image).

    Returns (app_ota_hash, full_serial_hash) or (None, None) if inputs missing.
    """
    import shutil
    import subprocess
    import sys
    import tempfile

    bootloader  = os.path.join(build_dir, "bootloader.bin")
    partitions  = os.path.join(build_dir, "partitions.bin")
    firmware    = os.path.join(build_dir, "firmware.bin")
    app_ota     = os.path.join(build_dir, "firmware_app_ota.bin")
    full_serial = os.path.join(build_dir, "firmware_full_serial.bin")

    if not all(os.path.isfile(f) for f in [bootloader, partitions, firmware]):
        print("[variant] merge skipped: bootloader/partitions/firmware not all present")
        return None, None

    shutil.copy2(firmware, app_ota)

    fd, tmp_merged = tempfile.mkstemp(prefix="fw_merged_", suffix=".bin", dir=build_dir)
    os.close(fd)
    python_exe = env.subst("$PYTHONEXE") or sys.executable
    uploader   = env.subst("$UPLOADER")
    flash_size = env.BoardConfig().get("upload.flash_size", "4MB")
    mcu        = env.BoardConfig().get("build.mcu", "esp32")
    # Classic ESP32 boots its bootloader from 0x1000; S3/C3 from 0x0. The merged
    # image always starts at 0x0 (classic gets 0xFF padding for the first 4 KB).
    bootloader_offset = "0x1000" if mcu == "esp32" else "0x0"
    cmd = [
        python_exe, uploader, "--chip", mcu,
        "merge_bin", "-o", tmp_merged,
        "--flash_size", flash_size, "--flash_mode", "dio",
        bootloader_offset, bootloader,
        "0x8000",  partitions,
        "0x10000", firmware,
    ]
    print("[variant] Building firmware_full_serial.bin ...")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"[variant] merge FAILED: {result.stderr}")
        try:
            os.remove(tmp_merged)
        except OSError:
            pass
        return None, None
    shutil.move(tmp_merged, full_serial)
    print(f"[variant] Wrote firmware_full_serial.bin ({os.path.getsize(full_serial):,} B)"
          + (f" / FIRMWARE_VERSION = {fw_version}" if fw_version else ""))

    # Stale-build guard: the version string must be present in both binaries.
    if fw_version:
        needle = fw_version.encode("ascii")
        for p in (app_ota, full_serial):
            with open(p, "rb") as f:
                if needle not in f.read():
                    raise RuntimeError(
                        f"[variant] VERSION MISMATCH: {os.path.basename(p)} lacks "
                        f"'{fw_version}' — stale build. Run `pio run -t clean`.")
    return _sha256(app_ota), _sha256(full_serial)


def write_variant(source, target, env):  # noqa: ARG001
    pioenv = env.subst("$PIOENV")
    base = VARIANTS.get(pioenv)
    if base is None:
        return
    build_dir = env.subst("$BUILD_DIR")
    fw, commit = _read_build_version(env.subst("$PROJECT_DIR"))

    # Merge first so we can hash the published bytes into variant.json.
    app_ota_hash, full_serial_hash = _merge_serial(env, build_dir, fw)

    meta = dict(base)
    if fw:
        meta["fwVersion"] = fw
    if commit:
        meta["buildCommit"] = commit
    if app_ota_hash:
        meta["appOtaHash"] = app_ota_hash
    if full_serial_hash:
        meta["fullSerialHash"] = full_serial_hash
    out = os.path.join(build_dir, "variant.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[variant] Wrote variant.json: role={meta['role']} "
          f"label={meta['label']} fwVersion={fw or '(none)'}")

    # Stable dist/ copy — `.pio/build` is volatile; dist/<env>/ (gitignored)
    # survives and is what the Studio dev plugin reads first.
    import shutil
    dist_dir = os.path.join(env.subst("$PROJECT_DIR"), "dist", pioenv)
    os.makedirs(dist_dir, exist_ok=True)
    for name in ("firmware.bin", "firmware_app_ota.bin",
                 "firmware_full_serial.bin", "variant.json"):
        src = os.path.join(build_dir, name)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(dist_dir, name))
    print(f"[variant] Copied distributables to dist/{pioenv}/")


# ---- NVS-safe serial upload (mirrors merge_firmware.py) --------------------
#   [0x0,     0x9000) bootloader + partition table   ← write
#   [0x9000,  0xE000) NVS (channel/mode/input_level) ← skip (preserve)
#   [0xE000,  0x10000) otadata                       ← erase to 0xFF (boot ota_0)
#   [0x10000, end   ) app                            ← write
def upload_split(source, target, env):  # noqa: ARG001
    import subprocess
    build_dir = env.subst("$BUILD_DIR")
    fw_path = os.path.join(build_dir, "firmware_full_serial.bin")
    if not os.path.isfile(fw_path):
        raise RuntimeError(f"upload_split: {fw_path} not found (build first)")
    with open(fw_path, "rb") as f:
        data = f.read()
    NVS_GAP_START, OTADATA_START, APP_START = 0x9000, 0xE000, 0x10000
    mcu = env.BoardConfig().get("build.mcu", "esp32")
    boot_magic_at = 0x1000 if mcu == "esp32" else 0x0
    if (len(data) < APP_START + 1
            or data[boot_magic_at] != 0xE9 or data[APP_START] != 0xE9):
        raise RuntimeError("upload_split: not the expected merged layout")
    head_path    = os.path.join(build_dir, "firmware.head.bin")
    otadata_path = os.path.join(build_dir, "firmware.otadata.bin")
    app_path     = os.path.join(build_dir, "firmware.app.bin")
    with open(head_path, "wb") as f:
        f.write(data[:NVS_GAP_START])
    with open(otadata_path, "wb") as f:
        f.write(b"\xFF" * (APP_START - OTADATA_START))
    with open(app_path, "wb") as f:
        f.write(data[APP_START:])
    cmd_str = env.subst(
        '"$PYTHONEXE" "$UPLOADER" $UPLOAD_FLAGS '
        f'write_flash 0x0 "{head_path}" '
        f'0x{OTADATA_START:X} "{otadata_path}" '
        f'0x{APP_START:X} "{app_path}"')
    print(f"[upload_split] {len(data[:NVS_GAP_START])} B @ 0x0 + 8192 B (0xFF) @ "
          f"0x{OTADATA_START:X} + {len(data[APP_START:])} B @ 0x{APP_START:X} "
          f"(NVS preserved)")
    rc = subprocess.call(cmd_str, shell=True)
    if rc != 0:
        raise RuntimeError(f"upload_split: esptool failed ({rc})")


env.Replace(UPLOADCMD=upload_split)  # noqa: F821
env.AddPostAction("$BUILD_DIR/firmware.bin", write_variant)  # noqa: F821
