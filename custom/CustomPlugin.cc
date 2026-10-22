#include "CustomPlugin.h"
#include "QGCLoggingCategory.h"

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

QGC_LOGGING_CATEGORY(CustomPluginLog, "Custom.Plugin")

Q_APPLICATION_STATIC(CustomPlugin, _customPluginInstance);

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

    // Start minimal health-check HTTP server on port 18080
    s_httpServer = std::make_unique<httplib::Server>();

    s_httpServer->Get("/healthz", [](const httplib::Request &, httplib::Response &res) {
        res.set_content(
            "{\"status\":\"ok\",\"service\":\"qgc-embedded-engine\"}\n",
            "application/json"
        );
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
}
