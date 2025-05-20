#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H


class TelemetryData {
public:
    TelemetryData() = default;

    // Initialize the telemetry data
    void init();

    // Update the telemetry data
    void update();

    // Get the current telemetry data
    const char* getData() const;





#endif // TelemetryData.h