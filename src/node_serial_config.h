#ifndef NODE_SERIAL_CONFIG_H
#define NODE_SERIAL_CONFIG_H

// ---------------------------------------------------------------------------
// node_serial_config.h — Studio JSON config for the transmitter (DEC-034).
//
// The transmitter is configured from Studio over USB Web Serial with the same
// JSON line protocol every node uses (contracts/specs/serial-config.md). It
// coexists with the binary Bridge frame protocol: a line starting with '{' is
// handled here; anything else is left for the binary SerialHandler.
//
// Supported: get_info (role=transmitter) / set_name / set_espnow_channel /
// set_input_level / set_input_mix / set_input_sel (EXPERIMENTAL, CoreS3 only) /
// set_stream_mode / set_relay_source / reboot.
// ---------------------------------------------------------------------------

// Call from loop() BEFORE the binary serial handler. Consumes a JSON config
// line if the next byte is '{'; otherwise returns without touching the stream.
void nodeSerialConfigUpdate();

#endif // NODE_SERIAL_CONFIG_H
