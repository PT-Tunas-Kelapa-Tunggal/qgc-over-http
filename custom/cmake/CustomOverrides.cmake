# ============================================================================
# Custom Build Configuration Overrides
# Embedded QGroundControl Engine Overrides
# ============================================================================

set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Target Apple Silicon" FORCE)
set(QGC_QT_MAXIMUM_VERSION "6.12.99" CACHE STRING "Maximum supported Qt version" FORCE)
