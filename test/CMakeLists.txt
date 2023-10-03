if(BUILD_TESTING)

    add_executable(data_loader_test data_loader_test.cpp)
    target_link_libraries(data_loader_test PRIVATE GTest::gtest_main data_loader)

    include(GoogleTest)
    gtest_discover_tests(data_loader_test)
endif()