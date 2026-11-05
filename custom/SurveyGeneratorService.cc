#include "SurveyGeneratorService.h"
#include "SurveySafetyPlanner.h"

#include "PlanMasterController.h"
#include "MissionController.h"
#include "MissionSettingsItem.h"
#include "SimpleMissionItem.h"
#include "SpeedSection.h"
#include "SurveyComplexItem.h"
#include "CameraCalc.h"
#include "QGCMapPolygon.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "QGCMAVLink.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QCoreApplication>
#include <QtPositioning/QGeoCoordinate>

#include <cmath>

QGC_LOGGING_CATEGORY(SurveyGeneratorServiceLog, "Custom.SurveyGeneratorService")

int SurveyGeneratorService::generateSurveyPlan(const std::string& requestJsonStr, std::string& responseJsonStr)
{
    QJsonParseError parseError;
    const QJsonDocument reqDoc = QJsonDocument::fromJson(QByteArray::fromStdString(requestJsonStr), &parseError);
    if (parseError.error != QJsonParseError::NoError || !reqDoc.isObject()) {
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Invalid JSON payload: ") + parseError.errorString();
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    const QJsonObject reqObj = reqDoc.object();
    const QJsonArray boundaryArray = reqObj.value(QStringLiteral("boundary")).toArray();
    if (boundaryArray.count() < 3) {
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Survey boundary requires at least 3 points");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    QList<QGeoCoordinate> boundaryCoords;
    boundaryCoords.reserve(boundaryArray.count());
    double sumLat = 0.0;
    double sumLon = 0.0;

    for (const QJsonValue& val : boundaryArray) {
        if (!val.isObject()) {
            QJsonObject errObj;
            errObj["status"] = QStringLiteral("error");
            errObj["message"] = QStringLiteral("Boundary elements must be coordinate objects");
            responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
            return 400;
        }

        const QJsonObject pt = val.toObject();
        const double lat = pt.value(QStringLiteral("latitude")).toDouble();
        const double lon = pt.value(QStringLiteral("longitude")).toDouble();

        if (!std::isfinite(lat) || !std::isfinite(lon) || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            QJsonObject errObj;
            errObj["status"] = QStringLiteral("error");
            errObj["message"] = QStringLiteral("Boundary coordinates out of range (-90..90, -180..180)");
            responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
            return 400;
        }

        boundaryCoords.append(QGeoCoordinate(lat, lon));
        sumLat += lat;
        sumLon += lon;
    }

    const QGeoCoordinate centerCoord(sumLat / boundaryCoords.count(), sumLon / boundaryCoords.count());

    const QJsonObject params = reqObj.value(QStringLiteral("parameters")).toObject();
    const double altitude = params.value(QStringLiteral("altitude")).toDouble(30.0);
    const double speed = params.value(QStringLiteral("speed")).toDouble(5.0);
    const double gridAngle = params.value(QStringLiteral("gridAngle")).toDouble(0.0);
    const double turnaroundDistance = params.value(QStringLiteral("turnaroundDistance")).toDouble(10.0);
    const bool hoverAndCapture = params.value(QStringLiteral("hoverAndCapture")).toBool(false);
    const bool useCameraOverlap = params.value(QStringLiteral("useCameraOverlap")).toBool(true);

    double frontalOverlap = params.value(QStringLiteral("frontalOverlap")).toDouble(75.0);
    double sideOverlap = params.value(QStringLiteral("sideOverlap")).toDouble(70.0);
    if (frontalOverlap > 0.0 && frontalOverlap <= 1.0) {
        frontalOverlap *= 100.0;
    }
    if (sideOverlap > 0.0 && sideOverlap <= 1.0) {
        sideOverlap *= 100.0;
    }

    const double spacing = params.value(QStringLiteral("spacing")).toDouble(20.0);
    const double triggerDistance = params.value(QStringLiteral("triggerDistance")).toDouble(15.0);
    const int entryLocation = params.value(QStringLiteral("entryLocation")).toInt(0);
    const int firmwareType = params.value(QStringLiteral("firmwareType")).toInt(MAV_AUTOPILOT_ARDUPILOTMEGA);
    const int vehicleType = params.value(QStringLiteral("vehicleType")).toInt(MAV_TYPE_QUADROTOR);
    const double ascentSpeed = params.value(QStringLiteral("ascentSpeed")).toDouble(2.5);
    const double descentSpeed = params.value(QStringLiteral("descentSpeed")).toDouble(1.5);

    // Defensive parameter checks
    if (!std::isfinite(altitude) || altitude <= 0.0 || !std::isfinite(speed) || speed <= 0.0 ||
        !std::isfinite(ascentSpeed) || ascentSpeed <= 0.0 || !std::isfinite(descentSpeed) || descentSpeed <= 0.0) {
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Altitude and speeds (flight, ascent, descent) must be positive finite numbers");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    SurveySafetyConfig safetyConfig;
    QString safetyError;
    if (!SurveySafetyPlanner::parseConfig(reqObj, params, centerCoord, altitude, turnaroundDistance, safetyConfig, safetyError)) {
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = safetyError;
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    // Configure offline editing defaults in AppSettings
    AppSettings* appSettings = SettingsManager::instance()->appSettings();
    if (appSettings) {
        if (appSettings->offlineEditingCruiseSpeed()) {
            appSettings->offlineEditingCruiseSpeed()->setRawValue(speed);
        }
        if (appSettings->offlineEditingHoverSpeed()) {
            appSettings->offlineEditingHoverSpeed()->setRawValue(speed);
        }
        if (appSettings->offlineEditingFirmwareClass()) {
            appSettings->offlineEditingFirmwareClass()->setRawValue(
                QGCMAVLink::firmwareClass(static_cast<MAV_AUTOPILOT>(firmwareType)));
        }
        if (appSettings->offlineEditingVehicleClass()) {
            appSettings->offlineEditingVehicleClass()->setRawValue(
                QGCMAVLink::vehicleClass(static_cast<MAV_TYPE>(vehicleType)));
        }
    }

    // Construct PlanMasterController for headless calculation
    PlanMasterController masterController;
    masterController.start();

    masterController.removeAll();

    MissionController* missionCtrl = masterController.missionController();
    if (!missionCtrl) {
        qCCritical(SurveyGeneratorServiceLog) << "MissionController unavailable in masterController";
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Internal error: MissionController unavailable");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    // Configure Planned Home Position & Horizontal Flight Speed on MissionSettingsItem
    missionCtrl->setHomePosition(safetyConfig.homeCoord);
    if (missionCtrl->visualItems() && missionCtrl->visualItems()->count() > 0) {
        MissionSettingsItem* settingsItem = missionCtrl->visualItems()->value<MissionSettingsItem*>(0);
        if (settingsItem && settingsItem->speedSection()) {
            settingsItem->speedSection()->setSpecifyFlightSpeed(true);
            if (settingsItem->speedSection()->flightSpeed()) {
                settingsItem->speedSection()->flightSpeed()->setRawValue(speed);
            }
        }
    }

    // Insert Takeoff at Home
    VisualMissionItem* takeoffVisual = missionCtrl->insertTakeoffItem(safetyConfig.homeCoord, -1);
    if (SimpleMissionItem* takeoffSimple = qobject_cast<SimpleMissionItem*>(takeoffVisual)) {
        if (takeoffSimple->altitude()) {
            takeoffSimple->altitude()->setRawValue(altitude);
        }
    }

    // Insert Ascent Speed (MAV_CMD_DO_CHANGE_SPEED, speed_type = 2: Climb Speed)
    if (SimpleMissionItem* climbSpeedItem = qobject_cast<SimpleMissionItem*>(
            missionCtrl->insertSimpleMissionItem(safetyConfig.homeCoord, -1))) {
        climbSpeedItem->setCommand(MAV_CMD_DO_CHANGE_SPEED);
        climbSpeedItem->missionItem().setParam1(2.0);
        climbSpeedItem->missionItem().setParam2(ascentSpeed);
        climbSpeedItem->missionItem().setParam3(-1.0);
        climbSpeedItem->missionItem().setParam4(0.0);
    }

    // Insert Descent Speed (MAV_CMD_DO_CHANGE_SPEED, speed_type = 3: Descent Speed)
    if (SimpleMissionItem* descentSpeedItem = qobject_cast<SimpleMissionItem*>(
            missionCtrl->insertSimpleMissionItem(safetyConfig.homeCoord, -1))) {
        descentSpeedItem->setCommand(MAV_CMD_DO_CHANGE_SPEED);
        descentSpeedItem->missionItem().setParam1(3.0);
        descentSpeedItem->missionItem().setParam2(descentSpeed);
        descentSpeedItem->missionItem().setParam3(-1.0);
        descentSpeedItem->missionItem().setParam4(0.0);
    }

    // Insert Survey Complex Item
    VisualMissionItem* complexItem = missionCtrl->insertComplexMissionItem(
        SurveyComplexItem::canonicalName, centerCoord, -1);
    SurveyComplexItem* surveyItem = qobject_cast<SurveyComplexItem*>(complexItem);
    if (!surveyItem) {
        qCCritical(SurveyGeneratorServiceLog) << "Failed to insert SurveyComplexItem";
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Internal error: Failed to instantiate SurveyComplexItem");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    // Set boundary vertices
    QGCMapPolygon* polygon = surveyItem->surveyAreaPolygon();
    if (!polygon) {
        qCCritical(SurveyGeneratorServiceLog) << "SurveyComplexItem surveyAreaPolygon is null";
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Internal error: Survey polygon model missing");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    polygon->clear();
    polygon->appendVertices(boundaryCoords);

    // Apply parameters to SurveyComplexItem
    if (surveyItem->gridAngle()) {
        surveyItem->gridAngle()->setRawValue(gridAngle);
    }
    if (surveyItem->turnAroundDistance()) {
        surveyItem->turnAroundDistance()->setRawValue(turnaroundDistance);
    }
    if (surveyItem->hoverAndCapture()) {
        surveyItem->hoverAndCapture()->setRawValue(hoverAndCapture);
    }

    CameraCalc* cameraCalc = surveyItem->cameraCalc();
    if (cameraCalc) {
        if (cameraCalc->valueSetIsDistance()) {
            cameraCalc->valueSetIsDistance()->setRawValue(true);
        }
        if (cameraCalc->distanceToSurface()) {
            cameraCalc->distanceToSurface()->setRawValue(altitude);
        }

        if (useCameraOverlap) {
            QString cameraName = params.value(QStringLiteral("cameraName")).toString();
            if (!cameraName.isEmpty() && cameraName != CameraCalc::canonicalCustomCameraName() && cameraName != CameraCalc::canonicalManualCameraName()) {
                cameraCalc->setCameraBrand(cameraName);
            } else {
                QJsonObject spec = params.value(QStringLiteral("cameraSpec")).toObject();
                // Default to iPhone 17 Main Camera (24MP mode)
                double sensorWidth = spec.value(QStringLiteral("sensorWidth")).toDouble(8.19);
                double sensorHeight = spec.value(QStringLiteral("sensorHeight")).toDouble(6.14);
                double focalLength = spec.value(QStringLiteral("focalLength")).toDouble(6.86);
                int imageWidth = spec.value(QStringLiteral("imageWidth")).toInt(5712);
                int imageHeight = spec.value(QStringLiteral("imageHeight")).toInt(4284);
                bool landscape = spec.value(QStringLiteral("landscape")).toBool(true);

                cameraCalc->setCameraBrand(CameraCalc::canonicalCustomCameraName());
                if (cameraCalc->sensorWidth()) {
                    cameraCalc->sensorWidth()->setRawValue(sensorWidth);
                }
                if (cameraCalc->sensorHeight()) {
                    cameraCalc->sensorHeight()->setRawValue(sensorHeight);
                }
                if (cameraCalc->focalLength()) {
                    cameraCalc->focalLength()->setRawValue(focalLength);
                }
                if (cameraCalc->imageWidth()) {
                    cameraCalc->imageWidth()->setRawValue(imageWidth);
                }
                if (cameraCalc->imageHeight()) {
                    cameraCalc->imageHeight()->setRawValue(imageHeight);
                }
                if (cameraCalc->landscape()) {
                    cameraCalc->landscape()->setRawValue(landscape);
                }
            }

            if (cameraCalc->frontalOverlap()) {
                cameraCalc->frontalOverlap()->setRawValue(frontalOverlap);
            }
            if (cameraCalc->sideOverlap()) {
                cameraCalc->sideOverlap()->setRawValue(sideOverlap);
            }
        } else {
            cameraCalc->setCameraBrand(CameraCalc::canonicalManualCameraName());
            if (cameraCalc->adjustedFootprintSide()) {
                cameraCalc->adjustedFootprintSide()->setRawValue(spacing);
            }
            if (cameraCalc->adjustedFootprintFrontal()) {
                cameraCalc->adjustedFootprintFrontal()->setRawValue(triggerDistance);
            }
        }
    }

    // Rotate entry point if specified (0: TopLeft, 1: TopRight, 2: BottomLeft, 3: BottomRight)
    if (entryLocation > 0) {
        for (int i = 0; i < (entryLocation % 4); ++i) {
            surveyItem->rotateEntryPoint();
        }
    }

    // Insert Return to Launch (when returnToHome is true) or Land at Home (when false)
    VisualMissionItem* endVisual = missionCtrl->insertLandItem(safetyConfig.homeCoord, -1);
    if (!safetyConfig.returnToHome) {
        if (SimpleMissionItem* endSimple = qobject_cast<SimpleMissionItem*>(endVisual)) {
            endSimple->setCommand(MAV_CMD_NAV_LAND);
        }
    }

    // Configure Geofence in QGC GeoFenceController
    SurveySafetyPlanner::applyGeoFence(masterController, boundaryCoords, surveyItem, safetyConfig);

    // Extract visual transect points
    QJsonArray visualPointsArray;
    const QVariantList transectPoints = surveyItem->visualTransectPoints();
    for (const QVariant& varPt : transectPoints) {
        if (varPt.canConvert<QGeoCoordinate>()) {
            const QGeoCoordinate coord = varPt.value<QGeoCoordinate>();
            QJsonObject ptObj;
            ptObj[QStringLiteral("latitude")] = coord.latitude();
            ptObj[QStringLiteral("longitude")] = coord.longitude();
            ptObj[QStringLiteral("altitude")] = coord.altitude();
            visualPointsArray.append(ptObj);
        }
    }

    // Flush event loop to allow queued signals to execute (such as mission flight status)
    QCoreApplication::processEvents();

    // Serialize full plan JSON and apply general waypoint hold duration + safety evaluation
    QJsonObject planObj = masterController.saveToJson().object();
    SurveySafetyStats safetyStats;
    const QJsonObject safetyObj = SurveySafetyPlanner::applyHoldAndEvaluateSafety(
        masterController, safetyConfig, planObj, safetyStats);

    const double surveyDist = surveyItem->complexDistance();
    double transitDist = 0.0;
    if (safetyConfig.homeCoord.isValid() && !transectPoints.isEmpty()) {
        const QGeoCoordinate firstPt = transectPoints.first().value<QGeoCoordinate>();
        const QGeoCoordinate lastPt = transectPoints.last().value<QGeoCoordinate>();
        if (firstPt.isValid()) {
            transitDist += safetyConfig.homeCoord.distanceTo(firstPt);
        }
        if (safetyConfig.returnToHome && lastPt.isValid()) {
            transitDist += lastPt.distanceTo(safetyConfig.homeCoord);
        }
    }

    const double climbDescentTime = (altitude / ascentSpeed) + (altitude / descentSpeed);
    const double horizontalTime = (surveyDist + transitDist) / speed;
    const double flightTime = horizontalTime + climbDescentTime + safetyStats.holdTimeSeconds;

    // Extract mission metrics
    QJsonObject statsObj;
    statsObj[QStringLiteral("totalDistanceMeters")] = surveyDist + transitDist;
    statsObj[QStringLiteral("flightTimeSeconds")] = flightTime;
    statsObj[QStringLiteral("coveredAreaSquareMeters")] = surveyItem->coveredArea();
    statsObj[QStringLiteral("photoCount")] = surveyItem->cameraShots();
    statsObj[QStringLiteral("transectCount")] = surveyItem->_transectCount();
    statsObj[QStringLiteral("waypointCount")] = safetyStats.waypointCount;
    statsObj[QStringLiteral("holdTimeSeconds")] = safetyStats.holdTimeSeconds;
    statsObj[QStringLiteral("maxMissionAltitude")] = safetyStats.maxMissionAltitude;
    if (cameraCalc) {
        statsObj[QStringLiteral("calculatedLineSpacing")] = cameraCalc->adjustedFootprintSide() ? cameraCalc->adjustedFootprintSide()->rawValue().toDouble() : 0.0;
        statsObj[QStringLiteral("calculatedTriggerDistance")] = cameraCalc->adjustedFootprintFrontal() ? cameraCalc->adjustedFootprintFrontal()->rawValue().toDouble() : 0.0;
        statsObj[QStringLiteral("groundResolutionCmPerPx")] = cameraCalc->imageDensity() ? cameraCalc->imageDensity()->rawValue().toDouble() : 0.0;
    }

    QJsonObject rootResponse;
    rootResponse[QStringLiteral("status")] = QStringLiteral("ok");
    rootResponse[QStringLiteral("statistics")] = statsObj;
    rootResponse[QStringLiteral("safety")] = safetyObj;
    rootResponse[QStringLiteral("visualTransectPoints")] = visualPointsArray;
    rootResponse[QStringLiteral("plan")] = planObj;

    responseJsonStr = QJsonDocument(rootResponse).toJson(QJsonDocument::Compact).toStdString();
    return 200;
}
