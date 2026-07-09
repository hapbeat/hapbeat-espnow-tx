// ---------------------------------------------------------------------------
// ima_adpcm.cpp — IMA ADPCM encoder (see header). Tables/algorithm identical
// to hapbeat-device-firmware/src/ima_adpcm.cpp so the receiver decodes us.
// ---------------------------------------------------------------------------

#include "ima_adpcm.h"

static const int16_t STEP_TABLE[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,
    16,    17,    19,    21,    23,    25,    28,    31,
    34,    37,    41,    45,    50,    55,    60,    66,
    73,    80,    88,    97,    107,   118,   130,   143,
    157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,
    724,   796,   876,   963,   1060,  1166,  1282,  1411,
    1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
    3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
    7132,  7845,  8630,  9493,  10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
};

static const int8_t INDEX_TABLE[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

void adpcmStateInit(AdpcmState* state) {
    state->predictor  = 0;
    state->step_index = 0;
}

// Internal: advance the state by "decoding" the nibble (keeps the encoder in
// lock-step with the receiver's decoder).
static void adpcmAdvance(uint8_t nibble, AdpcmState* state) {
    int step = STEP_TABLE[state->step_index];
    int diff = step >> 3;
    if (nibble & 1) diff += step >> 2;
    if (nibble & 2) diff += step >> 1;
    if (nibble & 4) diff += step;
    if (nibble & 8) diff = -diff;

    int pred = state->predictor + diff;
    if (pred > 32767)  pred = 32767;
    if (pred < -32768) pred = -32768;
    state->predictor = (int16_t)pred;

    int idx = state->step_index + INDEX_TABLE[nibble & 0x0F];
    if (idx < 0)  idx = 0;
    if (idx > 88) idx = 88;
    state->step_index = (uint8_t)idx;
}

uint8_t adpcmEncodeSample(int16_t sample, AdpcmState* state) {
    int step = STEP_TABLE[state->step_index];
    int diff = sample - state->predictor;

    uint8_t nibble = 0;
    if (diff < 0) {
        nibble = 8;
        diff = -diff;
    }
    if (diff >= step)        { nibble |= 4; diff -= step; }
    if (diff >= (step >> 1)) { nibble |= 2; diff -= (step >> 1); }
    if (diff >= (step >> 2)) { nibble |= 1; }

    adpcmAdvance(nibble, state);
    return nibble;
}

void adpcmEncodeBlockStereo(const int16_t* input, uint8_t* output,
                            size_t num_frames,
                            AdpcmState* state_l, AdpcmState* state_r) {
    for (size_t i = 0; i < num_frames; i++) {
        uint8_t lo = adpcmEncodeSample(input[i * 2],     state_l);
        uint8_t hi = adpcmEncodeSample(input[i * 2 + 1], state_r);
        output[i] = (hi << 4) | (lo & 0x0F);
    }
}

void adpcmEncodeBlockMono(const int16_t* input, uint8_t* output,
                          size_t num_samples, AdpcmState* state) {
    for (size_t i = 0; i < num_samples; i += 2) {
        uint8_t lo = adpcmEncodeSample(input[i], state);
        uint8_t hi = 0;
        if (i + 1 < num_samples) {
            hi = adpcmEncodeSample(input[i + 1], state);
        }
        output[i / 2] = (hi << 4) | (lo & 0x0F);
    }
}
