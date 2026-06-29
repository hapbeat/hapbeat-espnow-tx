#!/usr/bin/env node
/**
 * Generate manifest.json for a Hapbeat TRANSMITTER firmware GitHub Release
 * (DEC-035). Transmitter nodes are serial-only (no Wi-Fi OTA), so each env has
 * only a merged `<env>_firmware_full_serial.bin`.
 *
 * Usage:
 *   node generate-manifest.mjs <artifacts-dir> [tag]
 *
 * fwVersion comes from the per-env `<env>_variant.json` (baked by build_version.py
 * from firmware-versions.json), NOT the tag — tags are namespaced
 * (tx/<env>/vX.Y.Z). The serial image's sha256 (contentHash) is computed here
 * since the merge happens in CI (no pre-build hash like the device repo).
 * Output is fragment-shaped (validates against contracts
 * firmware-manifest-fragment.schema.json).
 */

import { createHash } from 'crypto'
import { readdir, readFile, stat, writeFile } from 'fs/promises'
import { dirname, join } from 'path'
import { fileURLToPath } from 'url'

const [artifactsDir, tag = 'unknown'] = process.argv.slice(2)
if (!artifactsDir) {
  console.error('Usage: generate-manifest.mjs <artifacts-dir> [tag]')
  process.exit(1)
}

const __dirname = dirname(fileURLToPath(import.meta.url))

function bareVersionFromTag(t) {
  if (!t || t === 'unknown') return ''
  return t.includes('/v') ? t.slice(t.lastIndexOf('/v') + 2) : t.replace(/^v/, '')
}

let versionsJson = {}
try {
  versionsJson = JSON.parse(
    await readFile(join(__dirname, '..', '..', 'firmware-versions.json'), 'utf-8'),
  ).envs ?? {}
} catch {
  versionsJson = {}
}

async function tryStat(p) {
  try { return await stat(p) } catch { return null }
}

async function sha256(p) {
  try {
    return 'sha256:' + createHash('sha256').update(await readFile(p)).digest('hex')
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
  if (f.endsWith(FULL_SERIAL_SUFFIX)) envSet.add(f.slice(0, -FULL_SERIAL_SUFFIX.length))
}

const variants = []
for (const env of [...envSet].sort()) {
  const fullSerialFile = `${env}${FULL_SERIAL_SUFFIX}`
  const st = await tryStat(join(artifactsDir, fullSerialFile))
  if (!st) continue

  const meta = (await tryReadVariantJson(env)) ?? {}
  const fwVersion = meta.fwVersion || versionsJson[env] || bareVersionFromTag(tag)
  const contentHash = await sha256(join(artifactsDir, fullSerialFile))

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
    hapbeat: false,
    fwVersion,
    ...(meta.buildCommit ? { buildCommit: meta.buildCommit } : {}),
    fullSerial: {
      filename: fullSerialFile,
      size: st.size,
      mtime: Math.round(st.mtimeMs),
      ...(contentHash ? { contentHash } : {}),
    },
  })
}

const manifest = { tag, schema_version: 3, variants }
const outPath = join(artifactsDir, 'manifest.json')
await writeFile(outPath, JSON.stringify(manifest, null, 2), 'utf-8')

console.log(`manifest.json written to ${outPath} (tag=${tag})`)
for (const v of variants) {
  console.log(`  [${v.role}/${v.transport}] ${v.env}: fw=${v.fwVersion} fullSerial=${!!v.fullSerial}`)
}
