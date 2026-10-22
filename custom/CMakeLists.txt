# ============================================================================
# Custom Plugin (Embedded QGC Engine)
# ============================================================================

message(STATUS "QGC: Adding Custom Plugin (Embedded Engine)")

if(APPLE)
    set(CUSTOM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.mm
        ${CMAKE_CURRENT_SOURCE_DIR}/CustomPlugin.h
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
        CACHE INTERNAL "" FORCE
    )
endif()

set(CUSTOM_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}
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
