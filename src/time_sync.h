#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <cstdint>

// Time synchronization from Bridge.
// Maintains an offset between the Bridge's microsecond clock and the local
// millis() clock so that ESP-NOW packets can carry a synchronized timestamp.
class TimeSync {
public:
    TimeSync();

    // Update the local time reference with the Bridge's current time (microseconds).
    // Called when a TIME_SYNC command is received.
    void updateBridgeTime(int64_t bridge_time_us);

    // Return current synchronized time in milliseconds.
    // If no sync has been received yet, returns local millis().
    uint32_t getCurrentTimeMs() const;

    // Return true if at least one sync has been received.
    bool isSynced() const;

private:
    bool    synced_;
    int64_t offset_ms_;       // bridge_time_ms - local_millis at sync point
    int64_t last_sync_local_; // local millis() at last sync
};

#endif // TIME_SYNC_H
