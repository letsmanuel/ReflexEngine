add_executable(reflex_demo main.cpp)
target_link_libraries(reflex_demo PRIVATE reflex raylib)
if(MSVC)
    target_compile_options(reflex_demo PRIVATE /W3)
    set_target_properties(reflex_demo PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

# assets/ (if any) are copied next to the executable
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
    add_custom_command(TARGET reflex_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:reflex_demo>/assets)
endif()
