#pragma once

#include <string>

class SurveyGeneratorService
{
public:
    /// Generates a complete QGC .plan document (including mission, speed/hold items, and geofence),
    /// transect visual points, safety validation report, and mission statistics.
    /// MUST be executed on the Qt Main Event Loop thread.
    ///
    /// @param requestJsonStr  Input JSON containing boundary, optional home/returnToHome, parameters, and geofence
    /// @param responseJsonStr Output JSON containing status, statistics, safety, visualTransectPoints, and native QGC .plan
    /// @return HTTP status code (200 on success, 400 on invalid input, 500 on internal failure)
    static int generateSurveyPlan(const std::string& requestJsonStr, std::string& responseJsonStr);
};
