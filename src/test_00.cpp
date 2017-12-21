#define CATCH_CONFIG_MAIN
#include "catch.hpp"

unsigned int Factorial(unsigned int number) {
  return number <= 1 ? 1 : Factorial(number-1)*number;
}

TEST_CASE("Factorial of 0 is 1 (fail", "[single-file]") {
  REQUIRE(Factorial(0) == 1);
}

TEST_CASE ("Factorials are computed", "[factorial]") {
  REQUIRE(Factorial(1) == 1);
  REQUIRE(Factorial(2) == 2);
  REQUIRE(Factorial(3) == 6);
  REQUIRE(Factorial(10) == 3628800);
}
