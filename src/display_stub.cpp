// ---------------------------------------------------------------------------
// display_stub.cpp — headless no-op display for the XIAO ESP32-C6 repeater.
//
// display.cpp (M5 LCD) is excluded from the xiao_c6_repeater env's
// build_src_filter (it needs M5Unified/M5Stack, which the XIAO has no display
// for). These stubs satisfy the display.h references on the shared setup()
// path (main.cpp) so the headless build links without an LCD. Empty on every
// other env, where the real display.cpp provides these symbols.
// ---------------------------------------------------------------------------

#ifdef BOARD_XIAO_C6

#include "display.h"

void displayInit() {}
void displayBootStatus(const char*) {}
void displayUpdateHeader(bool) {}
void displayUpdateStatus(uint32_t, uint32_t, uint32_t, uint8_t, bool) {}
void displaySendReaction(const char*, const char*, uint8_t, bool) {}
void displayError(const char*) {}
void displayUpdateAudioStats(uint8_t, int, uint32_t, int, uint32_t, uint32_t) {}
void displayUpdateRepeaterStats(uint8_t, const char*, uint32_t, uint32_t) {}

#endif // BOARD_XIAO_C6
