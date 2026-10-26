#include "CustomPlugin.h"
#include "QGCLoggingCategory.h"
#include "QGCApplication.h"
#include <QtQuick/QQuickWindow>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wshadow"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wsign-conversion"
#endif

#include "httplib.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#include <QtCore/QApplicationStatic>

#include <memory>
#include <thread>

#if defined(__APPLE__)
#import <Foundation/Foundation.h>
#endif

#include "SurveyGeneratorService.h"
#include <QtCore/QMetaObject>

QGC_LOGGING_CATEGORY(CustomPluginLog, "Custom.Plugin")

Q_APPLICATION_STATIC(CustomPlugin, _customPluginInstance);

#if defined(__APPLE__)
static id<NSObject> s_appNapActivity = nil;
#endif
static std::unique_ptr<std::thread> s_httpThread = nullptr;
static std::unique_ptr<httplib::Server> s_httpServer = nullptr;

CustomPlugin::CustomPlugin(QObject *parent)
    : QGCCorePlugin(parent)
{
    qCDebug(CustomPluginLog) << "CustomPlugin created";
}

CustomPlugin::~CustomPlugin()
{
    cleanup();
}

QGCCorePlugin *CustomPlugin::instance()
{
    return _customPluginInstance();
}

void CustomPlugin::init()
{
    QGCCorePlugin::init();
    _startServer();
    if (qEnvironmentVariableIsSet("QGC_HEADLESS")) {
        // Queue hiding the main root window right after it finishes initializing
        QMetaObject::invokeMethod(qGuiApp, []() {
            if (qGuiApp && qGuiApp->mainRootWindow()) {
                qGuiApp->mainRootWindow()->setVisible(false);
                qCDebug(CustomPluginLog) << "QGC headless mode active: mainRootWindow hidden";
            }
        }, Qt::QueuedConnection);
    }
}


void CustomPlugin::cleanup()
{
    _stopServer();
    QGCCorePlugin::cleanup();
}

void CustomPlugin::_startServer()
{
    if (_serverRunning) {
        return;
    }
    _serverRunning = true;

#if defined(__APPLE__)
    // Disable macOS App Nap so background MAVLink and HTTP are not throttled
    s_appNapActivity = [[NSProcessInfo processInfo]
        beginActivityWithOptions:(NSActivityUserInitiatedAllowingIdleSystemSleep | NSActivityLatencyCritical)
                          reason:@"ControlDroneMapping QGC Engine"];
    qCDebug(CustomPluginLog) << "macOS App Nap disabled via NSProcessInfo assertion";
#endif

    // Start minimal HTTP server on port 18080
    s_httpServer = std::make_unique<httplib::Server>();

    s_httpServer->Get("/healthz", [](const httplib::Request &, httplib::Response &res) {
        res.set_content(
            "{\"status\":\"ok\",\"service\":\"qgc-embedded-engine\"}\n",
            "application/json"
        );
    });

    s_httpServer->Post("/api/v1/plan/survey", [](const httplib::Request &req, httplib::Response &res) {
        std::string responseBody;
        int statusCode = 200;

        // Dispatched synchronously to the Qt main GUI thread
        QMetaObject::invokeMethod(qGuiApp, [&]() {
            statusCode = SurveyGeneratorService::generateSurveyPlan(req.body, responseBody);
        }, Qt::BlockingQueuedConnection);

        res.status = statusCode;
        res.set_content(responseBody, "application/json");
    });

    s_httpThread = std::make_unique<std::thread>([]() {
        qCDebug(CustomPluginLog) << "Starting embedded health-check HTTP server on 127.0.0.1:18080";
        if (s_httpServer) {
            s_httpServer->listen("127.0.0.1", 18080);
        }
        qCDebug(CustomPluginLog) << "Embedded health-check HTTP server stopped";
    });
}

void CustomPlugin::_stopServer()
{
    if (!_serverRunning) {
        return;
    }
    _serverRunning = false;

    if (s_httpServer) {
        s_httpServer->stop();
    }
    if (s_httpThread && s_httpThread->joinable()) {
        s_httpThread->join();
    }
    s_httpServer.reset();
    s_httpThread.reset();

#if defined(__APPLE__)
    if (s_appNapActivity) {
        [[NSProcessInfo processInfo] endActivity:s_appNapActivity];
        s_appNapActivity = nil;
        qCDebug(CustomPluginLog) << "macOS App Nap activity ended";
    }
#endif
}
