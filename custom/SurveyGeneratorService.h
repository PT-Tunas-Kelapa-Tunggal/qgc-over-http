#pragma once

#include <string>

class SurveyGeneratorService
{
public:
    /// Generates a complete QGC .plan document, transect visual points, and statistics.
    /// MUST be executed on the Qt Main Event Loop thread.
    ///
    /// @param requestJsonStr  Input JSON containing boundary points and parameters
    /// @param responseJsonStr Output JSON containing status, statistics, visualTransectPoints, and native QGC .plan
    /// @return HTTP status code (200 on success, 400 on invalid input, 500 on internal failure)
    static int generateSurveyPlan(const std::string& requestJsonStr, std::string& responseJsonStr);
};
