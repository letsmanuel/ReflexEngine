if(REFLEX_BUILD_SHARED)
    add_library(reflex SHARED)
else()
    add_library(reflex STATIC)
endif()
add_library(reflex::reflex ALIAS reflex)

target_sources(reflex PRIVATE
    types.cpp
    injury.cpp
    authority.cpp
    balance.cpp
    threat.cpp
    backend/jolt_backend.cpp)

target_include_directories(reflex PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(reflex PRIVATE Jolt)
target_compile_features(reflex PUBLIC cxx_std_20)

if(REFLEX_BUILD_SHARED)
    set_target_properties(reflex PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_compile_definitions(reflex PUBLIC REFLEX_SHARED)
    target_compile_definitions(reflex PRIVATE REFLEX_BUILDING)
endif()

if(MSVC)
    target_compile_options(reflex PRIVATE /W4 /permissive-)
else()
    target_compile_options(reflex PRIVATE -Wall -Wextra -Wpedantic)
endif()
