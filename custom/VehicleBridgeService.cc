#include "VehicleBridgeService.h"

#include "MultiVehicleManager.h"
#include "Vehicle.h"
#include "VehicleLinkManager.h"
#include "SysStatusSensorInfo.h"
#include "HealthAndArmingCheckReport.h"
#include "VehicleGPSFactGroup.h"
#include "BatteryFactGroupListModel.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

QGC_LOGGING_CATEGORY(VehicleBridgeServiceLog, "Custom.VehicleBridgeService")

namespace {
double safeFactDouble(Fact *fact, double fallback = 0.0)
{
    if (!fact) {
        return fallback;
    }
    const QVariant val = fact->rawValue();
    if (!val.isValid() || val.isNull()) {
        return fallback;
    }
    bool ok = false;
    const double d = val.toDouble(&ok);
    return ok ? d : fallback;
}

int safeFactInt(Fact *fact, int fallback = 0)
{
    if (!fact) {
        return fallback;
    }
    const QVariant val = fact->rawValue();
    if (!val.isValid() || val.isNull()) {
        return fallback;
    }
    bool ok = false;
    const int i = val.toInt(&ok);
    return ok ? i : fallback;
}
}

int VehicleBridgeService::getVehiclesJson(std::string &responseBody)
{
    MultiVehicleManager *mvm = MultiVehicleManager::instance();
    if (!mvm) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("MultiVehicleManager instance is unavailable");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    QmlObjectListModel *vehicles = mvm->vehicles();
    Vehicle *activeVehicle = mvm->activeVehicle();

    QJsonArray vehicleArray;
    if (vehicles) {
        for (int i = 0; i < vehicles->count(); ++i) {
            Vehicle *v = qobject_cast<Vehicle*>(vehicles->get(i));
            if (!v) {
                continue;
            }

            QJsonObject vObj;
            vObj["id"] = v->id();
            vObj["active"] = (activeVehicle && activeVehicle->id() == v->id());
            vObj["firmwareType"] = v->firmwareTypeString();
            vObj["vehicleType"] = v->vehicleTypeString();
            vObj["armed"] = v->armed();
            vObj["flying"] = v->flying();
            vObj["landing"] = v->landing();
            vObj["guidedMode"] = v->guidedMode();
            vObj["flightMode"] = v->flightMode();
            vObj["flightModes"] = QJsonArray::fromStringList(v->flightModes());
            vObj["readyToFly"] = v->readyToFly();
            vObj["readyToFlyAvailable"] = v->readyToFlyAvailable();
            vObj["allSensorsHealthy"] = v->allSensorsHealthy();
            vObj["communicationLost"] = v->vehicleLinkManager() ? v->vehicleLinkManager()->communicationLost() : false;

            HealthAndArmingCheckReport *armReport = v->healthAndArmingCheckReport();
            vObj["canArm"] = armReport ? armReport->canArm() : v->allSensorsHealthy();

            SysStatusSensorInfo *sensorInfo = qobject_cast<SysStatusSensorInfo*>(v->sysStatusSensorInfo());
            if (sensorInfo) {
                vObj["sensorNames"] = QJsonArray::fromStringList(sensorInfo->sensorNames());
                vObj["sensorStatus"] = QJsonArray::fromStringList(sensorInfo->sensorStatus());
            } else {
                vObj["sensorNames"] = QJsonArray();
                vObj["sensorStatus"] = QJsonArray();
            }

            // Live Telemetry
            QJsonObject telemObj;
            telemObj["latitude"] = v->coordinate().latitude();
            telemObj["longitude"] = v->coordinate().longitude();
            telemObj["altitudeRelative"] = safeFactDouble(v->altitudeRelative(), 0.0);
            telemObj["altitudeAMSL"] = safeFactDouble(v->altitudeAMSL(), 0.0);
            telemObj["groundSpeed"] = safeFactDouble(v->groundSpeed(), 0.0);
            telemObj["climbRate"] = safeFactDouble(v->climbRate(), 0.0);
            telemObj["heading"] = safeFactDouble(v->heading(), 0.0);
            telemObj["roll"] = safeFactDouble(v->roll(), 0.0);
            telemObj["pitch"] = safeFactDouble(v->pitch(), 0.0);
            telemObj["distanceToHome"] = safeFactDouble(v->distanceToHome(), 0.0);
            telemObj["missionCurrentSeq"] = safeFactInt(v->missionItemIndex(), 0);
            vObj["telemetry"] = telemObj;

            // Battery Fact Group
            QJsonObject batteryObj;
            batteryObj["voltage"] = 0.0;
            batteryObj["percent"] = 0.0;
            batteryObj["current"] = 0.0;
            batteryObj["temperature"] = 0.0;
            QmlObjectListModel *batteries = v->batteries();
            if (batteries && batteries->count() > 0) {
                BatteryFactGroup *bfg = qobject_cast<BatteryFactGroup*>(batteries->get(0));
                if (bfg) {
                    batteryObj["voltage"] = safeFactDouble(bfg->voltage(), 0.0);
                    batteryObj["percent"] = safeFactDouble(bfg->percentRemaining(), 0.0);
                    batteryObj["current"] = safeFactDouble(bfg->current(), 0.0);
                    batteryObj["temperature"] = safeFactDouble(bfg->temperature(), 0.0);
                }
            }
            vObj["battery"] = batteryObj;

            // GPS Fact Group
            QJsonObject gpsObj;
            gpsObj["satellites"] = 0;
            gpsObj["lock"] = 0;
            gpsObj["hdop"] = 0.0;
            VehicleGPSFactGroup *gpsGroup = qobject_cast<VehicleGPSFactGroup*>(v->gpsFactGroup());
            if (gpsGroup) {
                gpsObj["satellites"] = safeFactInt(gpsGroup->count(), 0);
                gpsObj["lock"] = safeFactInt(gpsGroup->lock(), 0);
                gpsObj["hdop"] = safeFactDouble(gpsGroup->hdop(), 0.0);
            }
            vObj["gps"] = gpsObj;

            // Link Fact Group
            QJsonObject linkObj;
            linkObj["rcRSSI"] = safeFactDouble(v->rcRSSI(), 0.0);
            vObj["link"] = linkObj;

            vehicleArray.append(vObj);
        }
    }

    QJsonObject root;
    root["status"] = QStringLiteral("ok");
    root["count"] = vehicleArray.count();
    root["vehicles"] = vehicleArray;

    responseBody = QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString();
    return 200;
}

int VehicleBridgeService::executeVehicleCommand(const std::string &requestBody, std::string &responseBody)
{
    QJsonParseError parseError;
    const QJsonDocument reqDoc = QJsonDocument::fromJson(QByteArray::fromStdString(requestBody), &parseError);
    if (parseError.error != QJsonParseError::NoError || !reqDoc.isObject()) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Invalid JSON payload: ") + parseError.errorString();
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    MultiVehicleManager *mvm = MultiVehicleManager::instance();
    if (!mvm) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("MultiVehicleManager instance is unavailable");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    const QJsonObject reqObj = reqDoc.object();
    const QString command = reqObj.value(QStringLiteral("command")).toString().toLower();
    const QJsonValue paramVal = reqObj.value(QStringLiteral("param"));

    Vehicle *targetVehicle = nullptr;
    if (reqObj.contains(QStringLiteral("vehicleId"))) {
        const int vehicleId = reqObj.value(QStringLiteral("vehicleId")).toInt();
        targetVehicle = mvm->getVehicleById(vehicleId);
    } else {
        targetVehicle = mvm->activeVehicle();
    }

    if (!targetVehicle) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("No connected vehicle available or target vehicleId not found");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 404;
    }

    qCDebug(VehicleBridgeServiceLog) << "Executing command:" << command << "on vehicle" << targetVehicle->id();

    if (command == QStringLiteral("set_active")) {
        mvm->setActiveVehicle(targetVehicle);
    } else if (command == QStringLiteral("arm")) {
        targetVehicle->setArmedShowError(true);
    } else if (command == QStringLiteral("disarm")) {
        targetVehicle->setArmedShowError(false);
    } else if (command == QStringLiteral("set_mode")) {
        const QString modeName = paramVal.toString();
        if (modeName.isEmpty()) {
            QJsonObject err;
            err["status"] = QStringLiteral("error");
            err["message"] = QStringLiteral("Missing or invalid 'param' (flight mode name) for set_mode command");
            responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
            return 400;
        }
        targetVehicle->setFlightMode(modeName);
    } else if (command == QStringLiteral("takeoff")) {
        const double alt = paramVal.isDouble() ? paramVal.toDouble() : 10.0;
        targetVehicle->guidedModeTakeoff(alt);
    } else if (command == QStringLiteral("rtl")) {
        targetVehicle->guidedModeRTL(false);
    } else if (command == QStringLiteral("pause") || command == QStringLiteral("hold")) {
        targetVehicle->pauseVehicle();
    } else if (command == QStringLiteral("land")) {
        targetVehicle->guidedModeLand();
    } else if (command == QStringLiteral("start_mission")) {
        targetVehicle->startMission();
    } else if (command == QStringLiteral("emergency_stop")) {
        targetVehicle->emergencyStop();
    } else {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Unknown command: ") + command;
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

int VehicleBridgeService::uploadPlanToVehicle(const std::string &requestBody, std::string &responseBody)
{
    QJsonParseError parseError;
    const QJsonDocument reqDoc = QJsonDocument::fromJson(QByteArray::fromStdString(requestBody), &parseError);
    if (parseError.error != QJsonParseError::NoError || !reqDoc.isObject()) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Invalid JSON payload: ") + parseError.errorString();
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    MultiVehicleManager *mvm = MultiVehicleManager::instance();
    if (!mvm) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("MultiVehicleManager is unavailable");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    const QJsonObject reqObj = reqDoc.object();
    Vehicle *targetVehicle = nullptr;
    if (reqObj.contains(QStringLiteral("vehicleId"))) {
        const int vehicleId = reqObj.value(QStringLiteral("vehicleId")).toInt();
        targetVehicle = mvm->getVehicleById(vehicleId);
    } else {
        targetVehicle = mvm->activeVehicle();
    }

    if (!targetVehicle) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("No connected vehicle available or target vehicleId not found");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 404;
    }

    QJsonObject planObj;
    if (reqObj.contains(QStringLiteral("plan")) && reqObj.value(QStringLiteral("plan")).isObject()) {
        planObj = reqObj.value(QStringLiteral("plan")).toObject();
    } else if (reqObj.contains(QStringLiteral("mission")) && reqObj.value(QStringLiteral("mission")).isObject()) {
        planObj = reqObj;
    } else {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Missing 'plan' object in payload");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 400;
    }

    const QByteArray planBytes = QJsonDocument(planObj).toJson(QJsonDocument::Compact);

    QTemporaryFile tempFile;
    if (!tempFile.open()) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Gagal membuat temporary file untuk plan");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }
    tempFile.write(planBytes);
    tempFile.flush();
    tempFile.close();

    PlanMasterController masterController;
    masterController.startStaticActiveVehicle(targetVehicle, false);
    masterController.loadFromFile(tempFile.fileName());

    MissionManager *missionMgr = targetVehicle->missionManager();
    if (!missionMgr) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Target vehicle mission manager is unavailable");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 500;
    }

    bool uploadFinished = false;
    bool uploadHasError = false;
    QString uploadErrMsg;

    QEventLoop loop;
    QTimer timer;
    timer.setSingleShot(true);
    timer.setInterval(8000); // 8 seconds timeout for MAVLink mission transfer

    auto connComplete = QObject::connect(missionMgr, &PlanManager::sendComplete, &loop,
        [&](bool error) {
            uploadFinished = true;
            uploadHasError = error;
            loop.quit();
        });

    auto connError = QObject::connect(missionMgr, &PlanManager::error, &loop,
        [&](int /*errorCode*/, const QString &errorMsg) {
            uploadHasError = true;
            uploadErrMsg = errorMsg;
        });

    auto connTimeout = QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

    timer.start();
    masterController.sendToVehicle();
    loop.exec();

    QObject::disconnect(connComplete);
    QObject::disconnect(connError);
    QObject::disconnect(connTimeout);

    if (!uploadFinished && !uploadHasError) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = QStringLiteral("Waktu pengunggahan misi ke drone habis (Timeout 8s)");
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 504;
    }

    if (uploadHasError) {
        QJsonObject err;
        err["status"] = QStringLiteral("error");
        err["message"] = uploadErrMsg.isEmpty() ? QStringLiteral("Gagal mengunggah misi ke drone") : uploadErrMsg;
        responseBody = QJsonDocument(err).toJson(QJsonDocument::Compact).toStdString();
        return 422;
    }

    QJsonObject resp;
    resp["status"] = QStringLiteral("ok");
    resp["message"] = QStringLiteral("Misi berhasil diunggah ke drone");
    resp["vehicleId"] = targetVehicle->id();
    responseBody = QJsonDocument(resp).toJson(QJsonDocument::Compact).toStdString();
    return 200;
}
