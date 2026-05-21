configure_file(../source/resources/external_control.script ${PROJECT_BINARY_DIR}/example/ COPYONLY)
configure_file(resource/input_recipe.txt ${PROJECT_BINARY_DIR}/example/ COPYONLY)
configure_file(resource/output_recipe.txt ${PROJECT_BINARY_DIR}/example/ COPYONLY)

file(GLOB SOURCES *.cpp)

find_package(Boost REQUIRED COMPONENTS program_options)

if(ELITE_COMPILE_KIN_PLUGIN)
    add_definitions(-DELITE_COMPILE_KIN_PLUGIN)
endif()

if(ELITE_COMPILE_POSE_ALG_PLUGIN)
    add_definitions(-DELITE_COMPILE_POSE_ALG_PLUGIN)
endif()

# Examples link the SDK shared library; ELITE_STATIC_LIBRARY must NOT be defined
# for shared builds. If static linking is introduced in the future, gate
# ELITE_STATIC_LIBRARY on that configuration.

foreach(SOURCE ${SOURCES})
    get_filename_component(ELITE_SDK_EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_executable(${ELITE_SDK_EXAMPLE_NAME} ${SOURCE})
    target_link_libraries(
        ${ELITE_SDK_EXAMPLE_NAME} 
        elite_cs_series_sdk::shared
        ${SYSTEM_LIB}
        Boost::program_options
    )
    target_link_directories(
        ${ELITE_SDK_EXAMPLE_NAME} 
        PRIVATE 
        ${CMAKE_BINARY_DIR}
        ${Boost_LIBRARY_DIRS}
    )
    target_include_directories(
        ${ELITE_SDK_EXAMPLE_NAME}
        PRIVATE 
        ${Boost_INCLUDE_DIRS}
    )
    if(WIN32)
        add_custom_command(
            TARGET ${ELITE_SDK_EXAMPLE_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:elite_cs_series_sdk_SHARED>
                    $<TARGET_FILE_DIR:${ELITE_SDK_EXAMPLE_NAME}>
        )
    endif()
endforeach()


