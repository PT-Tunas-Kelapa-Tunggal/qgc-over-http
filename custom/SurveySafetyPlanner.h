#pragma once

#include <QtCore/QJsonObject>
#include <QtCore/QList>
#include <QtPositioning/QGeoCoordinate>

class PlanMasterController;
class SurveyComplexItem;

struct SurveySafetyConfig
{
    bool enabled = true;
    QString fenceType = QStringLiteral("polygon");
    double bufferMeters = 20.0;
    double radiusMeters = 0.0;
    double ceilingMeters = 50.0;
    QString breachAction = QStringLiteral("rtl");
    bool hasCustomBreachReturn = false;
    QGeoCoordinate breachReturnCoord;
    double breachReturnAltitude = 30.0;
    double holdSeconds = 0.0;
    double flightAltitude = 30.0;
    QGeoCoordinate homeCoord;
    bool returnToHome = true;
};

struct SurveySafetyStats
{
    int waypointCount = 0;
    double holdTimeSeconds = 0.0;
    double maxMissionAltitude = 0.0;
};

class SurveySafetyPlanner
{
public:
    /// Parses optional geofence, hold, and home parameters from the request JSON.
    static bool parseConfig(
        const QJsonObject& reqObj,
        const QJsonObject& paramsObj,
        const QGeoCoordinate& fallbackHome,
        double altitude,
        double turnaroundDistance,
        SurveySafetyConfig& outConfig,
        QString& outError);

    /// Populates the GeoFenceController on PlanMasterController with the configured
    /// polygon or circular inclusion fence and breach return point.
    static void applyGeoFence(
        PlanMasterController& masterController,
        const QList<QGeoCoordinate>& boundaryCoords,
        SurveyComplexItem* surveyItem,
        SurveySafetyConfig& config);

    /// Post-processes the serialized .plan JSON to apply general waypoint hold duration
    /// and evaluates geofence coverage and ceiling limits against all mission items.
    static QJsonObject applyHoldAndEvaluateSafety(
        PlanMasterController& masterController,
        const SurveySafetyConfig& config,
        QJsonObject& planObj,
        SurveySafetyStats& outStats);
};
