//
// Created by vis75817 on 1/11/2022.
//
// enable catch2 main
#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
void TEST_CASE( "Test case 1", "[test]" ) {
    REQUIRE( 1 == 1 );
}