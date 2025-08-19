 * Original Author
 * @file LD2410S.h
 * @brief Driver library for LD2410S radar sensor (motion & distance)
 * @author PhuongNam720
 * @note License and original source can be found at https://github.com/phuongnamzz/HLK-LD2410S/
 * @version 1.0
 * @date 2025-07-03
---------------------------
 * Modified 8-18-2025
 * MillerOnTap
 * v2.0   

Summary of Changes from the original 1.0 HLK-LD2410S.cpp to LD2410S.cpp

Protocol engine refactor: Replaced ad-hoc UART writes/reads with a framed command I/O layer: sendCommand(), waitForAck*(), waitForData(), plus stateful parsers (feed(), feedData(), resetParser()). Cleaner header/tail handling and payload extraction.

Robust, non-blocking parsing: loop() now has a byte budget, header/footer re-sync, overflow protection, and an idle timeout—no busy waits, safer under RTOS load.

Event-driven data model: Minimal frames produce an LD2410SEvent (motion, distance, timestamp). Added getLatest(), wasUpdatedAndClear(), optional callback, and distance convenience getters (meters/feet).

API cleanup & naming:

Init pattern changed: constructor no longer wires pins; use begin(rxPin, txPin, baud) / end().

Renamed: writeGenericParametersCommand → writeGenericParameters, readCommonParametersCommand → readCommonParameters.

Removed direct state getters (isMotionDetected(), getDistance()) in favor of the event/latched API.

Configuration coverage expanded: High-level helpers for reading/writing common params; per-gate trigger/hold threshold R/W; automatic threshold tuning (autoUpdateThresholds) with progress reporting.

Retries & validation: All commands now support retries, timeouts, and strict ACK matching (with optional payload copy), improving reliability vs. fixed delays.

Diagnostics & logging: Bounded hex dumps (dumpFrame()), human-readable payload printing, ASCII and version decoders, and a parseLEUint32() utility promoted to a class member.

Backward compatibility: Core ops retained (enter/exitConfigMode, switchToMinimal/StandardMode, readFirmwareVersion, readSerialNumber) but implementations are stricter and non-blocking.
