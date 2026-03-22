#include "time_sync.h"
#include <Arduino.h>

TimeSync::TimeSync()
    : synced_(false)
    , offset_ms_(0)
    , last_sync_local_(0)
{
}

void TimeSync::updateBridgeTime(int64_t bridge_time_us) {
    int64_t bridge_time_ms = bridge_time_us / 1000;
    int64_t local_ms = static_cast<int64_t>(millis());

    offset_ms_ = bridge_time_ms - local_ms;
    last_sync_local_ = local_ms;
    synced_ = true;

    log_i("TimeSync updated: bridge_time_us=%lld, offset_ms=%lld", bridge_time_us, offset_ms_);
}

uint32_t TimeSync::getCurrentTimeMs() const {
    int64_t local_ms = static_cast<int64_t>(millis());
    if (!synced_) {
        return static_cast<uint32_t>(local_ms);
    }
    // Apply offset to convert local time to bridge-synchronized time
    int64_t synced_ms = local_ms + offset_ms_;
    return static_cast<uint32_t>(synced_ms & 0xFFFFFFFF);
}

bool TimeSync::isSynced() const {
    return synced_;
}
