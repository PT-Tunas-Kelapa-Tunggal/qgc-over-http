#pragma once

#include "QGCCorePlugin.h"

class CustomPlugin : public QGCCorePlugin
{
    Q_OBJECT

public:
    explicit CustomPlugin(QObject *parent = nullptr);
    ~CustomPlugin() override;

    static QGCCorePlugin *instance();

    void init() override;
    void cleanup() override;

private:
    void _startServer();
    void _stopServer();

    bool _serverRunning = false;
};
