#include "cli/format.hpp"

#include <catch2/catch_all.hpp>
#include <string>

TEST_CASE("format::DefaultFormat<bool>") {
  constexpr cli::format::DefaultFormat<bool> format;
  char buffer[32]{};
  SECTION("true value") {
    auto res = format(buffer, true);
    REQUIRE(res);
    REQUIRE(res.size_written == 4);
    REQUIRE(std::string((const char *)buffer) == "true");
  }
  SECTION("false value") {
    auto res = format(buffer, false);
    REQUIRE(res);
    REQUIRE(res.size_written == 5);
    REQUIRE(std::string((const char *)buffer) == "false");
  }
}
