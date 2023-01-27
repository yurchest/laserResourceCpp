set(SOURCES
    main.cpp
    control_laser.cpp
    control_laser.ui
    ${CMAKE_SOURCE_DIR}/png/png.qrc
    can.cpp
    gui_update.cpp
    settings_tab.cpp
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


target_link_libraries(Laser PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
   ${CMAKE_SOURCE_DIR}/lib/chai.dll
    )




# Сборка в папку LaserProgram
IF (WIN32)
    set_target_properties(Laser PROPERTIES
        MACOSX_BUNDLE TRUE
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
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/chai.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/ftd2xx.dll "${CMAKE_SOURCE_DIR}/LaserProgram/"

        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libgcc_s_seh-1.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libstdc++-6.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GCC_PATH}/libwinpthread-1.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/chai.dll "${CMAKE_CURRENT_BINARY_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/lib/ftd2xx.dll "${CMAKE_CURRENT_BINARY_DIR}/"
    )
ENDIF()
