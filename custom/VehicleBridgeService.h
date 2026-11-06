#pragma once

#include <string>

class VehicleBridgeService
{
public:
    /// Returns JSON listing all connected vehicles and their current telemetry,
    /// sensor health, battery, GPS, and flight status.
    /// MUST be called on the Qt main GUI thread.
    static int getVehiclesJson(std::string &responseBody);

    /// Executes a flight or mode command on a connected vehicle.
    /// Request JSON format:
    /// { "vehicleId": 1, "command": "arm|disarm|set_mode|takeoff|rtl|pause|land|start_mission|emergency_stop|set_active", "param": ... }
    /// MUST be called on the Qt main GUI thread.
    static int executeVehicleCommand(const std::string &requestBody, std::string &responseBody);
};
