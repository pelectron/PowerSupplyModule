#include "cli/cli.hpp"
#include "cli/function.hpp"
#include "cli/util.hpp"

// #include <cstdio>
// #include <source_location>

// template <typename T> void info() {
//   const std::source_location &loc = std::source_location::current();
//   std::printf("%s", loc.function_name());
// }

using namespace cli;

using cfg = cli::config;
struct MyCfg : cfg {

  cli::Error transmit(uint8_t c) {
    std::putchar(c);
    return {};
  }
};

static bool enable = false;

constexpr int free1(int param) { return param; }

constinit int virtual_ = 0;
static bool enable_virtual = false;

inline struct S {
  cli::Error free2(int x) { return {}; }
} s;

struct MyFunctor {
  cli::Error operator()(int x, char c) { return {}; }
};
struct MyFunctor2 {
  cli::Error operator()(float f) { return {}; }
};
void free3() { return; }
void free4(int) { return; }
int free5(void) { return -5; }

inline struct Settings {
  int a;
  int b;
  char c;
  void apply() {}
} settings;

constexpr auto n = cli::ctti::name<MyFunctor2>();
constexpr auto asda = "asdjfh"_sc;

constexpr MyCfg my_cfg{};
// clang-format off
static  cli::Cli my_cli(
    my_cfg,
    // functions
    // @{
    // free functions
    func("free1"_sc, &free1, "param"_a) ,
    // lambdas without templated call operator
    func("lambda"_sc, 
         [](int i, char c) {},
          "i"_a, 
          "c"_a),
    // and any other functor without templated call operator
    func("functor"_sc, MyFunctor{} ,"x"_a,"c"_a),
    func(MyFunctor2{}, "f"_a),
    // member functions
     func("free2"_sc, s, &S::free2, "x"_a),
  // @}
  // global objects
    param("enable"_sc,enable),
    // virtual hierarchies
    param("virtual"_sc,virtual_,param("enable"_sc, enable_virtual)),
    obj<settings>(
        // mem_data("a"_sc, &Settings::a),
        // mem_data("b"_sc, &Settings::b),
        // mem_data("c"_sc, &Settings::c),
        mem_fun<&Settings::apply>())

    );
// clang-format on

int main() {
  static constexpr auto name = ctti::value_name<&Settings::apply>();
  std::cout << std::string(name.data(), name.size()) << std::endl;
  // info<char>();
  my_cli.print();
  //   while (1) {
  //     my_cli.putc(static_cast<uint8_t>(std::getchar()));
  //   }
}
