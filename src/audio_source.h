#ifndef AUDIO_SOURCE_H
#define AUDIO_SOURCE_H

// ---------------------------------------------------------------------------
// audio_source.h — ESP-NOW live audio source (DEC-034, -D AUDIO_SOURCE)
//
// Captures PA line-in via an I2S codec (M5 Module-Audio / ES8388), decimates
// 48 kHz → 16 kHz, IMA-ADPCM-encodes, and broadcasts 0xAA stream packets over
// ESP-NOW for the espnow_stream receivers. See contracts/specs/espnow-stream.md.
//
// main.cpp dispatches setup()/loop() here when AUDIO_SOURCE is defined,
// instead of the Bridge command-relay path.
// ---------------------------------------------------------------------------

void audioSourceSetup();
void audioSourceLoop();

// Apply a new input level (0-100, 50 = unity) live, e.g. from Studio's
// set_input_level. CoreS3 maps it to the ES8388 analog PGA gain.
void audioSourceApplyInputLevel(int level);

// Persist one digital pre-encode gain step (0..8 = 0..+24 dB).
void audioSourceSetStreamGainStep(int step);

// Apply a new input mono-ize mode live (0=stereo, 1=mono_l, 2=mono_r), e.g.
// from Studio's set_input_mix. Workaround for the CoreS3 Module-Audio's
// hardware-defective L line-in channel (frequency-shaped attenuation that no
// flat trim can correct) — see the s_input_mix docstring in audio_source.cpp.
// No reboot needed: audioRxTask picks it up on the next captured sample.
void audioSourceApplyInputMix(int mix);

// Live Opus encoder complexity override (Studio's set_opus_complexity, mode 9
// SOLID48 tuning). value is 0-10 (OPUS_SET_COMPLEXITY) — a GLOBAL override
// applied to whichever Opus mode is currently streaming (all Opus modes share
// this one override; MODE_DEFS[mode].complexity is the per-mode default used
// when unset). Re-ctl's the running encoder immediately, no reboot. -1 clears
// the override. No-op (logged) on classic (non-CoreS3) senders, which have no
// Opus encoder. NVS persistence ("tx"/"opus_cplx") is the caller's job
// (node_serial_config.cpp), mirroring set_input_level/set_input_mix above.
void audioSourceApplyOpusComplexity(int value);
int  audioSourceGetOpusComplexity();

// EXPERIMENTAL hardware-debug command (Studio's set_input_sel). Hot-switches
// the ES8388 ADC input mux between its three addressable pins to investigate
// a frequency-shelf attenuation defect on the module's line-in L channel —
// see the s_input_sel docstring in audio_source.cpp for the full hypothesis
// and the sel index -> friendly-name mapping (in1/in2/diff1). CoreS3 only
// (returns false on classic Core); NOT persisted to NVS — a reboot always
// restores the compiled-in default (in1).
bool audioSourceApplyInputSel(int sel);
const char* audioSourceGetInputSelName();

// Live stream-mode select: 0=RAW/ADPCM, 1=FAST, 2=BALANCED, 3=SMOOTH,
// 4=STEREO, 5=HIFI. Persisted to NVS. Only ADPCM on the classic (non-CoreS3) sender.
void audioSourceSetMode(int mode);
int  audioSourceGetMode();

// LONGRANGE + fleet-tune (DEC-043 P5 / LR-family-ui). set_lr_preset (canonical,
// 0..5) / set_range_mode persist + reboot (CoreS3-only — the LR profile is Opus).
// set_lr_bitrate is a deprecated bitrate→preset alias. set_fleet_param broadcasts
// a 0xAC beacon (burst ×3 + 5 s resend). Getters feed get_info.
void audioSourceSetRange(bool long_range);
void audioSourceSetLrPreset(int preset);
void audioSourceSetLrBitrate(int bitrate);   // deprecated alias
void audioSourceSetFleetParam(int param, int value);
bool audioSourceGetRange();
int  audioSourceGetLrBitrate();
const char* audioSourceGetLrPreset();

// HP jitter-buffer depth (Studio's set_stream_hp_buffer, mode-9 SOLID48
// receiver tuning). `ms` is 40..500 (clamped); persists the raw ms to NVS
// ("tx"/"hp_buf_ms", owned internally — unlike input_level/mix, this mirrors
// the self-contained set_range_mode/set_lr_preset style since it must also
// drive the fleet mechanism) and broadcasts it to the fleet as 0xAC fleet-tune
// param 6 (wire = clamp(ms/10, 4, 50) deci-ms), reusing the exact same
// burst x3 + 5s-resend + per-param epoch mechanism as params 1-5
// (audioSourceSetFleetParam). audioSourceGetStreamHpBuffer returns the raw ms
// for get_info.
void audioSourceSetStreamHpBuffer(int ms);
int  audioSourceGetStreamHpBuffer();

#endif // AUDIO_SOURCE_H
