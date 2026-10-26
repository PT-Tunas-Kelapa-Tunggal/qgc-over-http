# ============================================================================
# Custom Plugin (Embedded QGC Engine)
# ============================================================================

message(STATUS "QGC: Adding Custom Plugin (Embedded Engine)")

if(APPLE)
    set(CUSTOM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.mm
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.h
        ${CMAKE_CURRENT_SOURCE_DIR}/SurveyGeneratorService.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/SurveyGeneratorService.h
        CACHE INTERNAL "" FORCE
    )
    set(CUSTOM_LIBRARIES
        "-framework Foundation"
        CACHE INTERNAL "" FORCE
    )

    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.mm PROPERTIES COMPILE_FLAGS "-fobjc-arc")

else()
    set(CUSTOM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.h
        ${CMAKE_CURRENT_SOURCE_DIR}/SurveyGeneratorService.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/SurveyGeneratorService.h
        CACHE INTERNAL "" FORCE
    )
endif()

set(CUSTOM_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/MissionManager
    ${CMAKE_SOURCE_DIR}/src/QmlControls
    ${CMAKE_SOURCE_DIR}/src/Vehicle
    ${CMAKE_SOURCE_DIR}/src/FactSystem
    ${CMAKE_SOURCE_DIR}/src/Settings
    ${CMAKE_SOURCE_DIR}/src/MAVLink
    CACHE INTERNAL "" FORCE
)

set(CUSTOM_DEFINITIONS
    QGC_CUSTOM_BUILD
    CUSTOMHEADER="CustomPlugin.h"
    CUSTOMCLASS=CustomPlugin
    CACHE INTERNAL "" FORCE
)

set(CUSTOM_QT_COMPONENTS
    Core
    CACHE INTERNAL "" FORCE
)
