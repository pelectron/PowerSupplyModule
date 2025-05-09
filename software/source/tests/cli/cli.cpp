#include "cli/cli.hpp"
#include "cli/ctti.hpp"
#include "cli/enums.hpp"
#include "cli/function.hpp"
#include "cli/parse.hpp"
#include "cli/util.hpp"
#include <cstdint>
#include <cstdio>
#include <ostream>
#include <string>
#include <vector>

// #include <cstdio>
// #include <source_location>

// template <typename T> void info() {
//   const std::source_location &loc = std::source_location::current();
//   std::printf("%s", loc.function_name());
// }
/*
 * cpp-terminal
 * C++ library for writing multi-platform terminal applications.
 *
 * SPDX-FileCopyrightText: 2019-2025 cpp-terminal
 *
 * SPDX-License-Identifier: MIT
 */

#include "cpp-terminal/exception.hpp"
#include "cpp-terminal/input.hpp"
#include "cpp-terminal/iostream.hpp"
#include "cpp-terminal/key.hpp"
#include "cpp-terminal/options.hpp"
#include "cpp-terminal/terminal.hpp"
#include "cpp-terminal/tty.hpp"
#include "cpp-terminal/version.hpp"

cli::Error stream(cli::ByteView s) {
  Term::cout << s << std::flush;
  return cli::Error::none;
}

using namespace cli;

using cfg = cli::config;
struct MyCfg : cfg {

  static cli::Error transmit(uint8_t c) {
    Term::cout << static_cast<char>(c) << std::flush;
    return {};
  }
};

static bool enable = false;

constexpr int free1(int param) { return param; }

constinit int virtual_ = 0;
static bool enable_virtual = false;
static constinit int enable_opts = 0xFF;

inline struct S {
  cli::Error free2(int x) { return {}; }
} s;

struct MyFunctor {
  cli::Error operator()(int x, char c) { return {}; }
};
struct MyFunctor2 {
  cli::Error operator()(int f) { return {}; }
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

constexpr MyCfg my_cfg{};
// clang-format off
static  cli::Cli my_cli(
    my_cfg, 
    cli::io::AnsiOutputHandler{&stream},
    // functions
    // @{
    // free functions
    func("free1"_sc, &free1, funcs::arg("param"_sc)) ,
    // lambdas without templated call operator
    func("lambda"_sc, 
         [](int i, char c) {},
          "i"_arg, 
          "c"_arg),
    // and any other functor without templated call operator
    func("functor"_sc, MyFunctor{} ,"x"_arg,"c"_arg),
    func(MyFunctor2{}, "f"_arg),
    // member functions
     func("free2"_sc, s, &S::free2, "x"_arg),
  // @}
  // global objects
    param("enable"_sc, enable),
    // virtual hierarchies
    param("virtual"_sc, virtual_,
          param("enable"_sc, enable_virtual,
          param("opts"_sc, enable_opts))),
    param<settings>(
        mem_data<&Settings::a>(),
        mem_data("b"_sc, &Settings::b),
        mem_data("c"_sc, &Settings::c),
        mem_fun<&Settings::apply>())

    );
// clang-format on

constinit cli::io::Input input(my_cli);

int main() {
  try {
    // check if the terminal is capable of handling input
    Term::terminal.setOptions(Term::Option::NoClearScreen,
                              Term::Option::NoSignalKeys, Term::Option::Cursor,
                              Term::Option::Raw);
    if (!Term::is_stdin_a_tty()) {
      throw Term::Exception("The terminal is not attached to a TTY and "
                            "therefore can't catch user input. Exiting...");
    }
    my_cli.print();
    while (1) {
      Term::Event event = Term::read_event();
      switch (event.type()) {
      case Term::Event::Type::Key: {
        Term::Key key(event);
        if (key == Term::Key::Ctrl_C)
          exit(0);
        input.on_char(key.value);
        break;
      }
      case Term::Event::Type::CopyPaste: {
        std::string key_str(event);
        if (!key_str.empty() && key_str[0] == '\033') {
          Term::cout << "You discovered a key combination not yet managed by "
                        "cpp-terminal (";
          for (std::size_t i = 0; i != key_str.size(); ++i) {
            Term::cout << static_cast<std::int32_t>(key_str[i]) << " ";
          }
          Term::cout << ").\nPlease report key combination pressed to "
                     << Term::homepage() << std::endl;
        }
      }
      default:
        break;
      }
      my_cli.process();
    }
  } catch (const Term::Exception &re) {
    Term::cerr << "cpp-terminal error: " << re.what() << std::endl;
    return 2;
  } catch (...) {
    Term::cerr << "Unknown error." << std::endl;
    return 1;
  }
  return 0;
}
