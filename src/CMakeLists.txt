set (bcrypt
        ../lib/bcrypt/src/bcrypt.cpp
        ../lib/bcrypt/src/blowfish.cpp
        )

set (chai
        ../lib/chai/src/chai-cbunp.c
        ../lib/chai/src/chai-lnx.c
        ../lib/chai/src/unican.h
        )

set(chai2
        ../lib/chai2/src/chai-lnx.c
#        ../lib/chai2/src/bmarathon.c
#        ../lib/chai2/src/boards.c
#        ../lib/chai2/src/boards-conf.c
#        ../lib/chai2/src/statstr.c
#        ../lib/chai2/src/unicandrv-lnx.c
#        ../lib/chai2/src/unicandrv.c
        )

set(SOURCES
        main.cpp
        control_laser.cpp
        control_laser.ui
        ${CMAKE_SOURCE_DIR}/png/png.qrc
        can.cpp
        gui_update.cpp
        settings_tab.cpp
        authentication.cpp
        ${bcrypt}
#        ${chai}
        )

# Set path to our custom CMAKE scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

# Include Qt deployment helper function
include(windeployqt)


FILE(GLOB_RECURSE HeaderFiles "${CMAKE_SOURCE_DIR}/src/*.h")
add_custom_target(headers SOURCES ${HeaderFiles})

add_executable(Laser
        ${SOURCES}
        )

set(LIBS
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets)

IF (WIN32)
    target_link_libraries(Laser PRIVATE ${LIBS} ${CMAKE_SOURCE_DIR}/lib/win/chai.dll)
ELSE ()
    target_link_libraries(Laser PRIVATE ${LIBS} ${CMAKE_SOURCE_DIR}/lib/unix/ftd2xx.a)
#    target_link_libraries(Laser PRIVATE ${LIBS} ftd2xx)
ENDIF ()

# Сборка в папку LaserProgram
IF (WIN32)
    set_target_properties(Laser PROPERTIES
            WIN32_EXECUTABLE TRUE
            )

    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/LaserProgram/)
    add_custom_command(TARGET Laser
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:Laser> "${CMAKE_SOURCE_DIR}/LaserProgram/")


    windeployqt(Laser)


    get_filename_component(GCC_PATH ${CMAKE_CXX_COMPILER} PATH)

    add_custom_command(TARGET Laser POST_BUILD

        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libgcc_s_seh-1.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libstdc++-6.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libwinpthread-1.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/win/chai.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/win/ftd2xx.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"

        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libgcc_s_seh-1.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libstdc++-6.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libwinpthread-1.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/win/chai.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/win/ftd2xx.dll "${CMAKE_CURRENT_BINARY_DIR}/"
    )
ENDIF()
