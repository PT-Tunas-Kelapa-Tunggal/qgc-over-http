#include "SurveyGeneratorService.h"

#include "PlanMasterController.h"
#include "MissionController.h"
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
    const double altitude = params.value(QStringLiteral("altitude")).toDouble(50.0);
    const double speed = params.value(QStringLiteral("speed")).toDouble(5.0);
    const double gridAngle = params.value(QStringLiteral("gridAngle")).toDouble(0.0);
    const double turnaroundDistance = params.value(QStringLiteral("turnaroundDistance")).toDouble(10.0);
    const bool hoverAndCapture = params.value(QStringLiteral("hoverAndCapture")).toBool(false);
    const bool useCameraOverlap = params.value(QStringLiteral("useCameraOverlap")).toBool(false);

    double frontalOverlap = params.value(QStringLiteral("frontalOverlap")).toDouble(70.0);
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

    // Defensive parameter checks
    if (!std::isfinite(altitude) || altitude <= 0.0 || !std::isfinite(speed) || speed <= 0.0) {
        QJsonObject errObj;
        errObj["status"] = QStringLiteral("error");
        errObj["message"] = QStringLiteral("Altitude and speed must be positive finite numbers");
        responseJsonStr = QJsonDocument(errObj).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    // Configure offline editing defaults in AppSettings
    AppSettings* appSettings = SettingsManager::instance()->appSettings();
    if (appSettings) {
        if (appSettings->offlineEditingCruiseSpeed()) {
            appSettings->offlineEditingCruiseSpeed()->setRawValue(speed);
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

    // Insert Takeoff
    missionCtrl->insertTakeoffItem(centerCoord, -1);

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

    // Insert Return to Launch / Land
    missionCtrl->insertLandItem(centerCoord, -1);

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

    // Extract mission metrics
    QJsonObject statsObj;
    statsObj[QStringLiteral("totalDistanceMeters")] = surveyItem->complexDistance();
    statsObj[QStringLiteral("flightTimeSeconds")] = missionCtrl->missionTime();
    statsObj[QStringLiteral("coveredAreaSquareMeters")] = surveyItem->coveredArea();
    statsObj[QStringLiteral("photoCount")] = surveyItem->cameraShots();
    statsObj[QStringLiteral("transectCount")] = surveyItem->_transectCount();

    // Serialize full plan JSON
    const QJsonDocument planDoc = masterController.saveToJson();

    QJsonObject rootResponse;
    rootResponse[QStringLiteral("status")] = QStringLiteral("ok");
    rootResponse[QStringLiteral("statistics")] = statsObj;
    rootResponse[QStringLiteral("visualTransectPoints")] = visualPointsArray;
    rootResponse[QStringLiteral("plan")] = planDoc.object();

    responseJsonStr = QJsonDocument(rootResponse).toJson(QJsonDocument::Compact).toStdString();
    return 200;
}
