#ifndef IMA_ADPCM_H
#define IMA_ADPCM_H

// ---------------------------------------------------------------------------
// ima_adpcm.h — IMA ADPCM encoder (transmitter side, DEC-034 audio source).
//
// Tables + algorithm are byte-identical to hapbeat-device-firmware's
// ima_adpcm so the receiver (espnow_stream.cpp) decodes correctly. Stereo
// packs 1 frame per byte: low nibble = L, high nibble = R.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstddef>

struct AdpcmState {
    int16_t predictor;
    uint8_t step_index;
};

void adpcmStateInit(AdpcmState* state);

// Encode one PCM16 sample → 4-bit nibble (advances state).
uint8_t adpcmEncodeSample(int16_t sample, AdpcmState* state);

// Encode interleaved stereo PCM16 (L,R,L,R,...) → 1 byte/frame
// (low nibble = L, high nibble = R). num_frames bytes written.
void adpcmEncodeBlockStereo(const int16_t* input, uint8_t* output,
                            size_t num_frames,
                            AdpcmState* state_l, AdpcmState* state_r);

// Encode a block of mono PCM16 to ADPCM (2 samples/byte: low nibble = sample N,
// high nibble = sample N+1). num_samples samples → ceil(num_samples/2) bytes.
// Byte-identical to the receiver's adpcmDecodeBlockMono unpacking
// (hapbeat-device-firmware) — used by mode 6 LITE.
void adpcmEncodeBlockMono(const int16_t* input, uint8_t* output,
                          size_t num_samples, AdpcmState* state);

#endif // IMA_ADPCM_H
