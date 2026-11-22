add_executable(reflex_sim sim.cpp)
target_include_directories(reflex_sim PRIVATE ${PROJECT_SOURCE_DIR}/tests)
target_link_libraries(reflex_sim PRIVATE reflex)
