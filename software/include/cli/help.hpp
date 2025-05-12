#ifndef CLI_HELP_HPP
#define CLI_HELP_HPP
#include "cli/command.hpp"
#include "cli/enums.hpp"
#include "cli/string.hpp"

namespace cli {

template <class Cli, class Cfg> struct Help {
  constexpr cli::Error operator()(ByteView cmd) const {
    if (cmd.size() == 0) {
      cli.print();
      return Error::none;
    }

    const CommandNode *node = &cli.root();
    auto end = cmd.find_first_of(Cfg::access_separator);
    while (end != ByteView::npos) {
      auto s = cmd.substr(0, end);
      bool found = false;
      for (const auto &sub : *node) {
        if (sub.name == s) {
          node = &sub;
          cmd = cmd.substr(end);
          end = cmd.find_last_of(Cfg::access_separator);
          found = true;
          break;
        }
      }
      if (not found)
        return Error::invalid_argument;
    }

    for (const auto &sub : *node) {
      if (sub.name == cmd) {
        node = &sub;
        cli.out_.write(sub.description);
        cli.out_.newline();
        return Error::none;
        break;
      }
    }

    return Error::invalid_argument;
  }

  Cli &cli;
};
} // namespace cli
#endif
