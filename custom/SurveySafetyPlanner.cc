#include "SurveySafetyPlanner.h"

#include "GeoFenceController.h"
#include "PlanMasterController.h"
#include "QGCFenceCircle.h"
#include "QGCFencePolygon.h"
#include "QGCLoggingCategory.h"
#include "QGCMAVLink.h"
#include "SurveyComplexItem.h"

#include <QtCore/QJsonArray>
#include <QtCore/QVariant>

#include <algorithm>
#include <cmath>

QGC_LOGGING_CATEGORY(SurveySafetyPlannerLog, "Custom.SurveySafetyPlanner")

bool SurveySafetyPlanner::parseConfig(
    const QJsonObject& reqObj,
    const QJsonObject& paramsObj,
    const QGeoCoordinate& fallbackHome,
    double altitude,
    double turnaroundDistance,
    SurveySafetyConfig& outConfig,
    QString& outError)
{
    outConfig.flightAltitude = altitude;
    outConfig.homeCoord = fallbackHome;
    outConfig.returnToHome = reqObj.value(QStringLiteral("returnToHome")).toBool(true);

    if (reqObj.contains(QStringLiteral("home")) && reqObj.value(QStringLiteral("home")).isObject()) {
        const QJsonObject homeObj = reqObj.value(QStringLiteral("home")).toObject();
        const double lat = homeObj.value(QStringLiteral("latitude")).toDouble( std::numeric_limits<double>::quiet_NaN());
        const double lon = homeObj.value(QStringLiteral("longitude")).toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(lat) || !std::isfinite(lon) || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            outError = QStringLiteral("Home coordinate out of valid range (-90..90, -180..180)");
            return false;
        }
        outConfig.homeCoord = QGeoCoordinate(lat, lon);
    }

    outConfig.holdSeconds = paramsObj.value(QStringLiteral("holdSeconds")).toDouble(0.0);
    if (!std::isfinite(outConfig.holdSeconds) || outConfig.holdSeconds < 0.0 || outConfig.holdSeconds > 600.0) {
        outError = QStringLiteral("Hold duration must be between 0 and 600 seconds");
        return false;
    }

    // Default geofence parameters
    outConfig.enabled = true;
    outConfig.fenceType = QStringLiteral("polygon");
    outConfig.bufferMeters = std::max(5.0, turnaroundDistance + 10.0);
    outConfig.radiusMeters = 0.0;
    outConfig.ceilingMeters = std::min(120.0, altitude + 20.0);
    outConfig.breachAction = QStringLiteral("rtl");
    outConfig.breachReturnCoord = outConfig.homeCoord;
    outConfig.breachReturnAltitude = altitude;

    if (reqObj.contains(QStringLiteral("geofence")) && reqObj.value(QStringLiteral("geofence")).isObject()) {
        const QJsonObject gfObj = reqObj.value(QStringLiteral("geofence")).toObject();
        outConfig.enabled = gfObj.value(QStringLiteral("enabled")).toBool(true);
        const QString reqType = gfObj.value(QStringLiteral("type")).toString(QStringLiteral("polygon")).toLower();
        if (reqType == QStringLiteral("circle") || reqType == QStringLiteral("circular")) {
            outConfig.fenceType = QStringLiteral("circle");
        } else {
            outConfig.fenceType = QStringLiteral("polygon");
        }

        if (gfObj.contains(QStringLiteral("bufferMeters"))) {
            outConfig.bufferMeters = gfObj.value(QStringLiteral("bufferMeters")).toDouble(outConfig.bufferMeters);
        }
        if (gfObj.contains(QStringLiteral("radiusMeters"))) {
            outConfig.radiusMeters = gfObj.value(QStringLiteral("radiusMeters")).toDouble(0.0);
        }
        if (gfObj.contains(QStringLiteral("ceilingMeters"))) {
            outConfig.ceilingMeters = gfObj.value(QStringLiteral("ceilingMeters")).toDouble(outConfig.ceilingMeters);
        }
        if (gfObj.contains(QStringLiteral("breachAction"))) {
            outConfig.breachAction = gfObj.value(QStringLiteral("breachAction")).toString(QStringLiteral("rtl")).toLower();
        }

        if (!std::isfinite(outConfig.bufferMeters) || outConfig.bufferMeters < 0.0 || outConfig.bufferMeters > 5000.0) {
            outError = QStringLiteral("Geofence buffer distance must be between 0 and 5000 meters");
            return false;
        }
        if (!std::isfinite(outConfig.radiusMeters) || outConfig.radiusMeters < 0.0 || outConfig.radiusMeters > 50000.0) {
            outError = QStringLiteral("Geofence radius must be between 0 and 50000 meters");
            return false;
        }
        if (!std::isfinite(outConfig.ceilingMeters) || outConfig.ceilingMeters <= 0.0 || outConfig.ceilingMeters > 1000.0) {
            outError = QStringLiteral("Geofence ceiling must be between 1 and 1000 meters");
            return false;
        }

        if (gfObj.contains(QStringLiteral("breachReturnPoint")) && gfObj.value(QStringLiteral("breachReturnPoint")).isObject()) {
            const QJsonObject brObj = gfObj.value(QStringLiteral("breachReturnPoint")).toObject();
            const double brLat = brObj.value(QStringLiteral("latitude")).toDouble(std::numeric_limits<double>::quiet_NaN());
            const double brLon = brObj.value(QStringLiteral("longitude")).toDouble(std::numeric_limits<double>::quiet_NaN());
            const double brAlt = brObj.value(QStringLiteral("altitude")).toDouble(altitude);
            if (std::isfinite(brLat) && std::isfinite(brLon) && brLat >= -90.0 && brLat <= 90.0 && brLon >= -180.0 && brLon <= 180.0) {
                outConfig.hasCustomBreachReturn = true;
                outConfig.breachReturnCoord = QGeoCoordinate(brLat, brLon);
                outConfig.breachReturnAltitude = std::isfinite(brAlt) && brAlt > 0.0 ? brAlt : altitude;
            }
        }
    }

    return true;
}

static QList<QGeoCoordinate> expandPolygonRadially(const QList<QGeoCoordinate>& vertices, const QGeoCoordinate& center, double bufferMeters)
{
    QList<QGeoCoordinate> expanded;
    expanded.reserve(vertices.count());
    for (const QGeoCoordinate& v : vertices) {
        const double dist = center.distanceTo(v);
        const double az = center.azimuthTo(v);
        expanded.append(center.atDistanceAndAzimuth(dist + bufferMeters, az));
    }
    return expanded;
}

void SurveySafetyPlanner::applyGeoFence(
    PlanMasterController& masterController,
    const QList<QGeoCoordinate>& boundaryCoords,
    SurveyComplexItem* surveyItem,
    SurveySafetyConfig& config)
{
    GeoFenceController* fenceCtrl = masterController.geoFenceController();
    if (!fenceCtrl) {
        qCWarning(SurveySafetyPlannerLog) << "GeoFenceController is unavailable";
        return;
    }

    fenceCtrl->removeAll();

    if (!config.enabled) {
        return;
    }

    if (config.fenceType == QStringLiteral("circle")) {
        double resolvedRadius = config.radiusMeters;
        if (resolvedRadius <= 0.0) {
            double maxDist = 0.0;
            for (const QGeoCoordinate& c : boundaryCoords) {
                maxDist = std::max(maxDist, config.homeCoord.distanceTo(c));
            }
            if (surveyItem) {
                const QVariantList transectVars = surveyItem->visualTransectPoints();
                for (const QVariant& v : transectVars) {
                    if (v.canConvert<QGeoCoordinate>()) {
                        const QGeoCoordinate tc = v.value<QGeoCoordinate>();
                        maxDist = std::max(maxDist, config.homeCoord.distanceTo(tc));
                    }
                }
            }
            resolvedRadius = std::ceil(maxDist + std::max(10.0, config.bufferMeters));
            config.radiusMeters = resolvedRadius;
        }

        QGCFenceCircle* circle = new QGCFenceCircle(config.homeCoord, resolvedRadius, true /* inclusion */, fenceCtrl);
        fenceCtrl->circles()->append(circle);
    } else {
        QGCFencePolygon* poly = new QGCFencePolygon(true /* inclusion */, fenceCtrl);
        poly->appendVertices(boundaryCoords);
        poly->verifyClockwiseWinding();
        const double initialArea = poly->area();
        const QGeoCoordinate polyCenter = poly->center();

        if (config.bufferMeters > 0.0) {
            poly->offset(config.bufferMeters);
            if (poly->count() < 3 || poly->area() <= initialArea) {
                poly->clear();
                poly->appendVertices(boundaryCoords);
                poly->verifyClockwiseWinding();
                poly->offset(-config.bufferMeters);
            }
            if (poly->count() < 3 || poly->area() <= initialArea) {
                poly->clear();
                poly->appendVertices(expandPolygonRadially(boundaryCoords, polyCenter, config.bufferMeters));
                poly->verifyClockwiseWinding();
            }
        }

        // Also ensure Home is inside when polygon fence is used; if not, keep as-is so coverage check flags it accurately
        fenceCtrl->polygons()->append(poly);
    }

    // Set Breach Return Point in QGC GeoFenceController
    QGeoCoordinate breachPt = config.hasCustomBreachReturn ? config.breachReturnCoord : config.homeCoord;
    if (breachPt.isValid()) {
        fenceCtrl->setBreachReturnPoint(breachPt);
        if (fenceCtrl->breachReturnAltitude()) {
            fenceCtrl->breachReturnAltitude()->setRawValue(config.breachReturnAltitude);
        }
    }
}

QJsonObject SurveySafetyPlanner::applyHoldAndEvaluateSafety(
    PlanMasterController& masterController,
    const SurveySafetyConfig& config,
    QJsonObject& planObj,
    SurveySafetyStats& outStats)
{
    outStats = SurveySafetyStats{};
    outStats.maxMissionAltitude = config.flightAltitude;

    QList<QGeoCoordinate> missionCoordinates;
    if (config.homeCoord.isValid()) {
        missionCoordinates.append(config.homeCoord);
    }

    // Traverse mission items in plan JSON to apply holdSeconds and collect coordinates/altitudes
    QJsonObject missionObj = planObj.value(QStringLiteral("mission")).toObject();
    QJsonArray topItems = missionObj.value(QStringLiteral("items")).toArray();

    for (int i = 0; i < topItems.count(); ++i) {
        QJsonObject itemObj = topItems.at(i).toObject();
        const QString itemType = itemObj.value(QStringLiteral("type")).toString();

        if (itemType == QStringLiteral("SimpleItem")) {
            const int cmd = itemObj.value(QStringLiteral("command")).toInt();
            QJsonArray params = itemObj.value(QStringLiteral("params")).toArray();
            if (params.count() >= 7) {
                const double lat = params.at(4).toDouble(std::numeric_limits<double>::quiet_NaN());
                const double lon = params.at(5).toDouble(std::numeric_limits<double>::quiet_NaN());
                const double alt = params.at(6).toDouble(0.0);
                if (std::isfinite(lat) && std::isfinite(lon) && (lat != 0.0 || lon != 0.0)) {
                    missionCoordinates.append(QGeoCoordinate(lat, lon, alt));
                }
                if (std::isfinite(alt) && alt > 0.0 &&
                    (cmd == MAV_CMD_NAV_WAYPOINT || cmd == MAV_CMD_NAV_TAKEOFF || cmd == MAV_CMD_NAV_LOITER_UNLIM)) {
                    outStats.maxMissionAltitude = std::max(outStats.maxMissionAltitude, alt);
                }
            }
        } else if (itemType == QStringLiteral("ComplexItem")) {
            QJsonObject transectObj = itemObj.value(QStringLiteral("TransectStyleComplexItem")).toObject();
            QJsonArray subItems = transectObj.value(QStringLiteral("Items")).toArray();

            for (int j = 0; j < subItems.count(); ++j) {
                QJsonObject subItem = subItems.at(j).toObject();
                const int cmd = subItem.value(QStringLiteral("command")).toInt();
                QJsonArray params = subItem.value(QStringLiteral("params")).toArray();

                if (params.count() >= 7) {
                    if (cmd == MAV_CMD_NAV_WAYPOINT) {
                        outStats.waypointCount += 1;
                        if (config.holdSeconds > 0.0) {
                            params.replace(0, config.holdSeconds);
                            subItem[QStringLiteral("params")] = params;
                            subItems.replace(j, subItem);
                        }
                    }

                    const double lat = params.at(4).toDouble(std::numeric_limits<double>::quiet_NaN());
                    const double lon = params.at(5).toDouble(std::numeric_limits<double>::quiet_NaN());
                    const double alt = params.at(6).toDouble(0.0);
                    if (std::isfinite(lat) && std::isfinite(lon) && (lat != 0.0 || lon != 0.0)) {
                        missionCoordinates.append(QGeoCoordinate(lat, lon, alt));
                    }
                    if (std::isfinite(alt) && alt > 0.0) {
                        outStats.maxMissionAltitude = std::max(outStats.maxMissionAltitude, alt);
                    }
                }
            }

            transectObj[QStringLiteral("Items")] = subItems;
            itemObj[QStringLiteral("TransectStyleComplexItem")] = transectObj;
            topItems.replace(i, itemObj);
        }
    }

    outStats.holdTimeSeconds = static_cast<double>(outStats.waypointCount) * config.holdSeconds;
    missionObj[QStringLiteral("items")] = topItems;
    planObj[QStringLiteral("mission")] = missionObj;

    // Evaluate geofence coverage & ceiling using QGC GeoFenceController models
    GeoFenceController* fenceCtrl = masterController.geoFenceController();
    bool coversMission = false;
    bool homeInside = false;
    int uncoveredCount = 0;

    QJsonArray polygonJsonArray;
    QJsonObject circleJsonObj;

    if (config.enabled && fenceCtrl) {
        if (config.fenceType == QStringLiteral("circle") && fenceCtrl->circles()->count() > 0) {
            QGCFenceCircle* circle = fenceCtrl->circles()->value<QGCFenceCircle*>(0);
            if (circle) {
                const QGeoCoordinate center = circle->center();
                const double radius = circle->radius()->rawValue().toDouble();

                QJsonObject centerObj;
                centerObj[QStringLiteral("latitude")] = center.latitude();
                centerObj[QStringLiteral("longitude")] = center.longitude();
                circleJsonObj[QStringLiteral("center")] = centerObj;
                circleJsonObj[QStringLiteral("radiusMeters")] = radius;

                homeInside = !config.homeCoord.isValid() || (center.distanceTo(config.homeCoord) <= radius);
                for (const QGeoCoordinate& pt : missionCoordinates) {
                    if (center.distanceTo(pt) > radius) {
                        uncoveredCount += 1;
                    }
                }
                coversMission = (uncoveredCount == 0) && homeInside;
            }
        } else if (fenceCtrl->polygons()->count() > 0) {
            QGCFencePolygon* poly = fenceCtrl->polygons()->value<QGCFencePolygon*>(0);
            if (poly) {
                const QList<QGeoCoordinate> verts = poly->coordinateList();
                for (const QGeoCoordinate& v : verts) {
                    QJsonObject vObj;
                    vObj[QStringLiteral("latitude")] = v.latitude();
                    vObj[QStringLiteral("longitude")] = v.longitude();
                    polygonJsonArray.append(vObj);
                }

                homeInside = !config.homeCoord.isValid() || poly->containsCoordinate(config.homeCoord);
                for (const QGeoCoordinate& pt : missionCoordinates) {
                    if (!poly->containsCoordinate(pt)) {
                        uncoveredCount += 1;
                    }
                }
                coversMission = (uncoveredCount == 0);
            }
        }
    }

    const bool ceilingOk = !config.enabled || (outStats.maxMissionAltitude <= config.ceilingMeters);

    QJsonArray issuesArray;
    if (!config.enabled) {
        QJsonObject issue;
        issue[QStringLiteral("code")] = QStringLiteral("GEOFENCE_DISABLED");
        issue[QStringLiteral("message")] = QStringLiteral("Geofence belum diaktifkan untuk rencana terbang ini.");
        issuesArray.append(issue);
    } else {
        if (!homeInside) {
            QJsonObject issue;
            issue[QStringLiteral("code")] = QStringLiteral("HOME_OUTSIDE_GEOFENCE");
            issue[QStringLiteral("message")] = QStringLiteral(
                "Titik Home berada di luar batas Geofence; perbesar buffer geofence atau pindahkan titik Home.");
            issuesArray.append(issue);
        }
        if (!coversMission) {
            QJsonObject issue;
            issue[QStringLiteral("code")] = QStringLiteral("GEOFENCE_COVERAGE_INSUFFICIENT");
            issue[QStringLiteral("message")] = QStringLiteral(
                "Area geofence belum mencakup seluruh jalur terbang dan manuver putar balik (%1 titik di luar batas).")
                                                   .arg(uncoveredCount);
            issuesArray.append(issue);
        }
        if (!ceilingOk) {
            QJsonObject issue;
            issue[QStringLiteral("code")] = QStringLiteral("CEILING_EXCEEDED");
            issue[QStringLiteral("message")] = QStringLiteral(
                "Ketinggian misi (%1 m) melebihi batas ketinggian maksimum / Ceiling geofence (%2 m).")
                                                   .arg(QString::number(outStats.maxMissionAltitude, 'f', 0),
                                                        QString::number(config.ceilingMeters, 'f', 0));
            issuesArray.append(issue);
        }
    }

    QJsonObject safetyObj;
    safetyObj[QStringLiteral("enabled")] = config.enabled;
    safetyObj[QStringLiteral("type")] = config.fenceType;
    safetyObj[QStringLiteral("bufferMeters")] = config.bufferMeters;
    safetyObj[QStringLiteral("radiusMeters")] = config.radiusMeters;
    safetyObj[QStringLiteral("ceilingMeters")] = config.ceilingMeters;
    safetyObj[QStringLiteral("breachAction")] = config.breachAction;
    safetyObj[QStringLiteral("coversMission")] = coversMission;
    safetyObj[QStringLiteral("homeInside")] = homeInside;
    safetyObj[QStringLiteral("uncoveredPointCount")] = uncoveredCount;
    safetyObj[QStringLiteral("ceilingOk")] = ceilingOk;
    safetyObj[QStringLiteral("polygon")] = polygonJsonArray;
    if (!circleJsonObj.isEmpty()) {
        safetyObj[QStringLiteral("circle")] = circleJsonObj;
    }

    const QGeoCoordinate brCoord = config.hasCustomBreachReturn ? config.breachReturnCoord : config.homeCoord;
    if (brCoord.isValid()) {
        QJsonObject brObj;
        brObj[QStringLiteral("latitude")] = brCoord.latitude();
        brObj[QStringLiteral("longitude")] = brCoord.longitude();
        brObj[QStringLiteral("altitude")] = config.breachReturnAltitude;
        safetyObj[QStringLiteral("breachReturnPoint")] = brObj;
    }

    safetyObj[QStringLiteral("issues")] = issuesArray;
    return safetyObj;
}
