#include "catch2/catch_test_macros.hpp"
#include "cli/format.hpp"

#include <catch2/catch_all.hpp>
#include <string>

TEST_CASE("format::String") {
  SECTION("unquoted strings") {
    std::string buffer(255, 0);
    using Formatter = cli::format::String<std::string_view, false>;
    auto res = Formatter{}({(uint8_t *)buffer.data(), buffer.size()}, "hello");
    REQUIRE(res.size_written == 5);
    buffer.resize(res.size_written);
    REQUIRE(buffer == std::string("hello"));
  }
  SECTION("quoted strings") {
    std::string buffer(255, 0);
    using Formatter = cli::format::String<std::string_view, true>;
    auto res = Formatter{}({(uint8_t *)buffer.data(), buffer.size()}, "hello");
    REQUIRE(res.size_written == 7);
    buffer.resize(res.size_written);
    REQUIRE(buffer == std::string("\"hello\""));
  }
}
