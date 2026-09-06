# TPMS — Reverse Engineering of Generic BLE Sensors

Component of **ESPelemetry** responsible for capturing, decoding, and exposing readings from 4 low-cost BLE TPMS (Tire Pressure Monitoring System) sensors, with no manufacturer-documented protocol.

## Context

The sensors used are generic external TPMS which transmit their readings as non-connectable BLE *advertising* — a GATT connection is never established. The ESP32-S3 operates in **GAP Observer role**, scanning passively and continuously.

The manufacturer does not publish the payload format. All byte mapping documented below was obtained through **field reverse engineering**: real captures, controlled physical tests, and cross-validation against the official manufacturer app (HRTPMS).

## Methodology

1. **Raw capture**: Arduino sketch (ESP32 + BLE library) filtering by the MAC address of each sensor, printing the complete *manufacturer data* in hexadecimal.
2. **Controlled reference generation**: Known pressure applied with a graduated syringe (volume/pressure ratio calculable by Boyle's Law) and variation of ambient temperature, comparing the captured raw data against the simultaneous reading from the official manufacturer app.
3. **Cross-validation**: Repetition of the process on the 4 sensors separately and simultaneously, to confirm that the formulas generalize across units and are not a single-sample coincidence.
4. **Hypothesis rejection by evidence**: Each initial assumption (based on protocols from other BLE TPMS manufacturers found in public forums and repositories) was tested against real data before being accepted.

## Payload structure (11 bytes)

```
Byte:     0    1    2    3    4    5    6    7    8    9   10
Field:  [ctr][00][seq][temp][---pressure---][-----ID-----][checksum?]
```

| Bytes | Field | Status | Detail |
|---|---|---|---|
| 0 | Counter/status | ⚠️ Unresolved | Varies with no clear correlation to pressure/temperature. Likely part of an integrity mechanism, not physical data. |
| 1 | Fixed (`0x00`) | ✅ Confirmed | Constant across the ~200 captured readings. |
| 2 | Slow counter | ✅ Confirmed | Increments slowly over time, independent of physical readings. Ignored in the decoder. |
| 3 | **Temperature** | ✅ Confirmed | `temp_C = byte[3] − 50`. Validated with real transitions (25°C→26°C) on all 4 sensors simultaneously. |
| 4–5 | **Pressure** | ✅ Confirmed (approx.) | 16-bit big-endian. `pressure_bar ≈ (raw − 100) × 0.01043`. Offset and slope derived by regression with 2–3 reference points per sensor; the high byte is only activated above ~1.5 bar. |
| 6–8 | **Sensor ID** | ✅ Confirmed | Matches exactly with the last 3 bytes of the sensor's MAC address. |
| 9–10 | Checksum/CRC | ⚠️ Unresolved | Modulo-256 sum and simple XOR ruled out. Changes non-linearly with any variation in the payload — consistent with a 16-bit CRC, but the exact algorithm was not identified. Not used to validate integrity. |
| — | **Battery** | ❌ Discarded from payload | Confirmed, by capturing the *complete* advertising packet (not just manufacturer data), that battery information **does not travel in the BLE packet**. The official app obtains it through another means outside the scope of a passive Observer. |

## Precision and known limitations

- The pressure formula is a **linear approximation** derived from a few reference points, not a factory calibration. The manufacturer itself declares a ±0.2 bar tolerance in its official app; the margin of error of this reimplementation is, at a minimum, equivalent.
- The zero point (offset) varies slightly among the 4 physical units — normal in low-cost sensors, not compensated by individual calibration.
- Without checksum verification, a packet corrupted by radio interference could be accepted without detection. Risk accepted given the use case (informative telemetry, not critical control).
- Temperature was only validated in the ambient range (~20–29°C); there is no confirmation of the formula at the extremes of the sensor's operating range.

## Component architecture

Linear pipeline, triggered by the BLE scan callback:

```
BLE Advertising → tpms_lookup_by_addr()   (which tire is it?)
                → tpms_raw_mfg_payload()  (bytes → raw values)
                → tpms_last_update()      (raw → physical, saved in tpms_storage[])
```

- `tpms_storage[]` is the shared state between the NimBLE host task (which writes on every received packet) and the rest of the application (which queries via `tpms_get_data()`), protected by a FreeRTOS mutex.
- Each function has a single responsibility: identifying the tire, decoding bytes, or persisting the result — none knows about the previous or next step more than strictly necessary.
- *Fail-soft* design: an invalid packet or one from an unknown device is discarded without affecting the last known valid data of each tire.

## Quick API reference

```c
esp_err_t tpms_init(void);
esp_err_t tpms_get_data(tpms_t tire, tpms_data_t *out_data);
float     tpms_bar_to_psi(float pressure_bar);
```

`tpms_get_data()` is thread-safe and returns a consistent copy of the last known state of the requested tire (`tire`, `pressure_bar`, `temp_c`, `last_seen_us`).

## Inspiration and sources

This project emerged as an initiative starting from the tutorial video by **upir**, which motivated the exploration of BLE TPMS sensors with ESP32:

- Video: [upir — Arduino TPMS Tire Pressure](https://www.youtube.com/watch?v=P85tkCbQGo8)
- Repository: [upiir/arduino_tpms_tire_pressure](https://github.com/upiir/arduino_tpms_tire_pressure)

During the reverse engineering stage, two third-party implementations for other BLE TPMS (with different payload lengths) were consulted as a comparative reference. Not directly applicable, but useful for contrasting structural patterns:

- [andi38/TPMS](https://github.com/andi38/TPMS)
- [ra6070/BLE-TPMS](https://github.com/ra6070/BLE-TPMS)
