configure_file(../source/resources/external_control.script ${PROJECT_BINARY_DIR}/example/ COPYONLY)
configure_file(resource/input_recipe.txt ${PROJECT_BINARY_DIR}/example/ COPYONLY)
configure_file(resource/output_recipe.txt ${PROJECT_BINARY_DIR}/example/ COPYONLY)

file(GLOB SOURCES *.cpp)

# On Windows systems, when using CMake in the main directory, 
# the ELITE_EXPORT_LIBRARY macro definition must be removed to avoid compilation errors.
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    remove_definitions(-DELITE_EXPORT_LIBRARY)
endif()

foreach(SOURCE ${SOURCES})
    get_filename_component(ELITE_SDK_EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_executable(${ELITE_SDK_EXAMPLE_NAME} ${SOURCE})
    target_link_libraries(
        ${ELITE_SDK_EXAMPLE_NAME} 
        elite-cs-series-sdk::shared
        ${SYSTEM_LIB}
    )
    target_link_directories(
        ${ELITE_SDK_EXAMPLE_NAME} 
        PRIVATE ${CMAKE_BINARY_DIR}
    )
endforeach()


