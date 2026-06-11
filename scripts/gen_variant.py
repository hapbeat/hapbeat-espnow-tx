"""Post-build: emit .pio/build/<env>/variant.json (role/transport/board/label).

Studio's firmware library groups builds by node role (DEC-034,
contracts/specs/firmware-distribution.md). Without this file the dev plugin
falls back to env-name inference, which can't classify names like "esp32"
and would misfile the Bridge relay under the Hapbeat (receiver) group.
"""
import json
import os

Import("env")  # noqa: F821  (PlatformIO construction environment)

VARIANTS = {
    # Bridge command relay: PC (Bridge) -> USB serial -> ESP-NOW commands.
    "esp32": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_core",
        "label": "Bridge 中継機 (ESP-NOW)",
        "description": "Bridge からのコマンドを ESP-NOW で Hapbeat 群に同報する中継機。",
    },
    # Live audio source: PA line-in -> ADPCM -> ESP-NOW 0xAA stream.
    "m5stack_audio_tx": {
        "role": "transmitter",
        "transport": "espnow_stream",
        "board": "m5stack_core",
        "label": "ESP-NOW ライブ送信機 (PA 入力)",
        "description": "PA ライン入力を ESP-NOW で同報するライブ音声送信機。",
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


env.AddPostAction("$BUILD_DIR/firmware.bin", write_variant)  # noqa: F821
