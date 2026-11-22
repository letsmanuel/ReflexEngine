add_executable(reflex_tests
    test_main.cpp
    test_backend.cpp
    test_injury.cpp
    test_authority.cpp
    test_balance.cpp
    test_threat.cpp
    test_behavior.cpp
    test_scenarios.cpp)

target_link_libraries(reflex_tests PRIVATE reflex doctest::doctest)
if(MSVC)
    target_compile_options(reflex_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(reflex_tests PRIVATE -Wall -Wextra)
endif()

add_test(NAME reflex_tests COMMAND reflex_tests)
