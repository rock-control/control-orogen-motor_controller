# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(motor_controllerTaskLib)
ADD_LIBRARY(${MOTOR_CONTROLLER_TASKLIB_NAME} SHARED 
    ${MOTOR_CONTROLLER_TASKLIB_SOURCES})
add_dependencies(${MOTOR_CONTROLLER_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${MOTOR_CONTROLLER_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${MOTOR_CONTROLLER_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${MOTOR_CONTROLLER_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${MOTOR_CONTROLLER_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${MOTOR_CONTROLLER_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${MOTOR_CONTROLLER_TASKLIB_HEADERS}
    DESTINATION include/orocos/motor_controller)

