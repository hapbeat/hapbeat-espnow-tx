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

// Live stream-mode select: 0=RAW/ADPCM, 1=FAST, 2=BALANCED, 3=SMOOTH,
// 4=STEREO, 5=HIFI. Persisted to NVS. Only ADPCM on the classic (non-CoreS3) sender.
void audioSourceSetMode(int mode);
int  audioSourceGetMode();

// LONGRANGE + fleet-tune (DEC-043 P5). set_range_mode / set_lr_bitrate persist +
// reboot (range is CoreS3-only — the LR profile is Opus). set_fleet_param
// broadcasts a 0xAC beacon (burst ×3 + 5 s resend). Getters feed get_info.
void audioSourceSetRange(bool long_range);
void audioSourceSetLrBitrate(int bitrate);
void audioSourceSetFleetParam(int param, int value);
bool audioSourceGetRange();
int  audioSourceGetLrBitrate();

#endif // AUDIO_SOURCE_H
