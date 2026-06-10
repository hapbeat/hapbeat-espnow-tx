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

#endif // AUDIO_SOURCE_H
