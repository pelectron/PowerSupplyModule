#include "catch2/catch_all.hpp"
#include "cli/parse.hpp"
#include "cli/util.hpp"
#include "common.hpp"
#include <type_traits>
using Seq1 = cli::FixedSizeVector<int, 10>;
using Seq2 = std::vector<int>;
using Result = cli::parse::ParseResult<std::string_view>;

template <class Seq> struct SeqTestVector {
  std::string_view input;
  cli::parse::ParseResult<Seq> output;
};

#define PASS_TV(...)                                                           \
  SeqTestVector<Seq> {                                                         \
    .input = "[" #__VA_ARGS__ "]", .output = { Seq{__VA_ARGS__}, {} }          \
  }

#define PASS_TV1(...)                                                          \
  SeqTestVector<Seq> {                                                         \
    .input = "[ " #__VA_ARGS__ "]", .output = { Seq{__VA_ARGS__}, {} }         \
  }

#define PASS_TV2(...)                                                          \
  SeqTestVector<Seq> {                                                         \
    .input = "[" #__VA_ARGS__ " ]", .output = { Seq{__VA_ARGS__}, {} }         \
  }

#define PASS_TV3(...)                                                          \
  SeqTestVector<Seq> {                                                         \
    .input = "[ " #__VA_ARGS__ " ]", .output = { Seq{__VA_ARGS__}, {} }        \
  }

#define PASS(...)                                                              \
  PASS_TV(__VA_ARGS__), PASS_TV1(__VA_ARGS__), PASS_TV2(__VA_ARGS__),          \
      PASS_TV3(__VA_ARGS__)

#define PASS_TVR(...)                                                          \
  SeqTestVector<Seq> {                                                         \
    .input = "[" #__VA_ARGS__ "]rest", .output = {Seq{__VA_ARGS__}, "rest"}    \
  }

#define PASS_TV1R(...)                                                         \
  SeqTestVector<Seq> {                                                         \
    .input = "[ " #__VA_ARGS__ "]rest", .output = {Seq{__VA_ARGS__}, "rest"}   \
  }

#define PASS_TV2R(...)                                                         \
  SeqTestVector<Seq> {                                                         \
    .input = "[" #__VA_ARGS__ " ]rest", .output = {Seq{__VA_ARGS__}, "rest"}   \
  }

#define PASS_TV3R(...)                                                         \
  SeqTestVector<Seq> {                                                         \
    .input = "[ " #__VA_ARGS__ " ]rest", .output = {Seq{__VA_ARGS__}, "rest"}  \
  }

#define PASSR(...)                                                             \
  PASS_TVR(__VA_ARGS__), PASS_TV1R(__VA_ARGS__), PASS_TV2R(__VA_ARGS__),       \
      PASS_TV3R(__VA_ARGS__)

#define str(x) std::string(x)

TEMPLATE_TEST_CASE("parse::Sequence", "", Seq1, Seq2) {
  using Seq = TestType;
  using Parser = cli::parse::DefaultParse<Seq>;

  Parser parse;
  SECTION("valid sequences") {
    // clang-format off
    SeqTestVector<Seq> vectors[]={
      PASS(),
      PASS(1,2,3,4),
      PASS(1 ,2,3,4),
      PASS(1 , 2,3,4),
      PASS(1,2,3, 4),
      PASS(1 ,2,3, 4),
      PASS(1 , 2,3, 4),
      PASS(1,2,3 ,4),
      PASS(1 ,2,3 ,4),
      PASS(1 , 2,3 ,4),
      PASS(1,2,3 , 4),
      PASS(1 ,2,3 , 4),
      PASS(1 , 2,3 , 4),
      PASS(1,2, 3,4),
      PASS(1 ,2, 3,4),
      PASS(1 , 2, 3,4),
      PASS(1,2, 3, 4),
      PASS(1 ,2, 3, 4),
      PASS(1 , 2 ,3, 4),
      PASS(1,2, 3 ,4),
      PASS(1 ,2, 3 ,4),
      PASS(1 , 2, 3 ,4),
      PASS(1,2, 3 , 4),
      PASS(1 ,2, 3 , 4),
      PASS(1 , 2, 3 , 4),
      PASSR(),
      PASSR(1,2,3,4),
      PASSR(1 ,2,3,4),
      PASSR(1 , 2,3,4),
      PASSR(1,2,3, 4),
      PASSR(1 ,2,3, 4),
      PASSR(1 , 2,3, 4),
      PASSR(1,2,3 ,4),
      PASSR(1 ,2,3 ,4),
      PASSR(1 , 2,3 ,4),
      PASSR(1,2,3 , 4),
      PASSR(1 ,2,3 , 4),
      PASSR(1 , 2,3 , 4),
      PASSR(1,2, 3,4),
      PASSR(1 ,2, 3,4),
      PASSR(1 , 2, 3,4),
      PASSR(1,2, 3, 4),
      PASSR(1 ,2, 3, 4),
      PASSR(1 , 2 ,3, 4),
      PASSR(1,2, 3 ,4),
      PASSR(1 ,2, 3 ,4),
      PASSR(1 , 2, 3 ,4),
      PASSR(1,2, 3 , 4),
      PASSR(1 ,2, 3 , 4),
      PASSR(1 , 2, 3 , 4)
    };
    // clang-format on

    for (const auto &tv : vectors) {
      auto res = parse(tv.input);
      REQUIRE(res);
      REQUIRE(res.error == tv.output.error);
      REQUIRE(res.value == tv.output.value);
      REQUIRE(str(res.rest) == str(tv.output.rest));
    }
  }
}
