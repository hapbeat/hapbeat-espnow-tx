#!/usr/bin/env node
/**
 * Generate manifest.json for a Hapbeat TRANSMITTER firmware GitHub Release.
 *
 * Usage:
 *   node generate-manifest.mjs <artifacts-dir> [tag]
 *
 * Transmitter nodes are serial-only (never join Wi-Fi → no Wi-Fi OTA), so each
 * env has only a merged `<env>_firmware_full_serial.bin` (no app-only OTA
 * image). Per-env `<env>_variant.json` (emitted by scripts/gen_variant.py)
 * carries role/transport/board/label. Mirrors
 * hapbeat-device-firmware/.github/scripts/generate-manifest.mjs, trimmed to the
 * serial-only shape.
 *
 * Output schema (v2 variants — what hapbeat-studio's
 * aggregate-firmware-manifest.mjs reads):
 * {
 *   "tag": "v0.1.0",
 *   "schema_version": 2,
 *   "variants": [
 *     {
 *       "id": "tx/m5stack_audio_tx",
 *       "repo": "hapbeat-transmitter-firmware",
 *       "env": "m5stack_audio_tx",
 *       "role": "transmitter",
 *       "transport": "espnow_stream",
 *       "board": "m5stack_basic",
 *       "label": "M5Stack Basic (ライブ送信機)",
 *       "hapbeat": false,
 *       "fwVersion": "0.1.0",
 *       "fullSerial": { "filename": "m5stack_audio_tx_firmware_full_serial.bin", "size": 1234, "mtime": 1714000000000 }
 *     }
 *   ]
 * }
 */

import { readdir, readFile, stat, writeFile } from 'fs/promises'
import { join } from 'path'

const [artifactsDir, tag = 'unknown'] = process.argv.slice(2)

if (!artifactsDir) {
  console.error('Usage: generate-manifest.mjs <artifacts-dir> [tag]')
  process.exit(1)
}

/** Stat a file, returning null if it does not exist. */
async function tryStat(p) {
  try {
    return await stat(p)
  } catch {
    return null
  }
}

async function tryReadVariantJson(env) {
  try {
    return JSON.parse(await readFile(join(artifactsDir, `${env}_variant.json`), 'utf-8'))
  } catch {
    return null
  }
}

const FULL_SERIAL_SUFFIX = '_firmware_full_serial.bin'
const files = await readdir(artifactsDir)

const envSet = new Set()
for (const f of files) {
  if (f.endsWith(FULL_SERIAL_SUFFIX)) {
    envSet.add(f.slice(0, -FULL_SERIAL_SUFFIX.length))
  }
}

const variants = []
for (const env of [...envSet].sort()) {
  const fullSerialFile = `${env}${FULL_SERIAL_SUFFIX}`
  const st = await tryStat(join(artifactsDir, fullSerialFile))
  if (!st) continue

  // Every transmitter env is an ESP-NOW sender peripheral; variant.json is the
  // source of truth for role/transport/board/label, with a static fallback.
  const meta = (await tryReadVariantJson(env)) ?? {}
  variants.push({
    id: `tx/${env}`,
    repo: 'hapbeat-transmitter-firmware',
    env,
    role: meta.role ?? 'transmitter',
    transport: meta.transport ?? 'espnow_stream',
    ...(meta.transports ? { transports: meta.transports } : {}),
    board: meta.board ?? 'm5stack_basic',
    label: meta.label ?? env,
    ...(meta.description ? { description: meta.description } : {}),
    // Transmitters are ecosystem peripherals (周辺機器), never wearables.
    // Studio groups its library by this flag, not by role.
    hapbeat: false,
    // fwVersion must match the device's reported version (tag without 'v').
    fwVersion: tag.replace(/^v/, ''),
    fullSerial: {
      filename: fullSerialFile,
      size: st.size,
      mtime: st.mtimeMs,
    },
  })
}

const manifest = { tag, schema_version: 2, variants }
const outPath = join(artifactsDir, 'manifest.json')
await writeFile(outPath, JSON.stringify(manifest, null, 2), 'utf-8')

console.log(`manifest.json written to ${outPath}`)
for (const v of variants) {
  console.log(`  [${v.role}/${v.transport}] ${v.env}: fullSerial=${!!v.fullSerial}`)
}
