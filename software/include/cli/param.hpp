#ifndef CLI_PARAM_HPP
#define CLI_PARAM_HPP

#include "cli/ctti.hpp"
#include "cli/enums.hpp"
#include "cli/format.hpp"
#include "cli/function.hpp"
#include "cli/parse.hpp"
#include "cli/type_list.hpp"
#include "cli/util.hpp"
#include <concepts>
#include <type_traits>

namespace cli::params {

template <class T> struct getter_value_type {
  using type = std::remove_cvref_t<
      type_list::type_at_t<0, typename function_traits<T>::arguments>>;
};

template <class T> struct setter_value_type {
  using type = std::remove_cvref_t<
      type_list::type_at_t<0, typename function_traits<T>::arguments>>;
};

template <class G, class V>
concept GetterOf = requires(G &&getter, V &value) {
  { getter(value) } -> std::same_as<Error>;
};

template <typename S, typename V>
concept SetterOf = requires(S &&setter, const V &value) {
  { setter(value) } -> std::same_as<Error>;
};

template <class G>
concept Getter = not std::same_as<void, typename getter_value_type<G>::type>;

template <class S>
concept Setter = not std::same_as<void, typename setter_value_type<S>::type>;

struct NullGet {
  constexpr NullGet() = default;
  constexpr NullGet(const NullGet &) = default;
  constexpr NullGet(NullGet &&) = default;
  constexpr NullGet &operator=(const NullGet &) = default;
  constexpr NullGet &operator=(NullGet &&) = default;
  constexpr Error operator()(dummy &) { return Error::none; }
};

struct NullSet {
  constexpr NullSet() = default;
  constexpr NullSet(const NullSet &) = default;
  constexpr NullSet(NullSet &&) = default;
  constexpr NullSet &operator=(const NullSet &) = default;
  constexpr NullSet &operator=(NullSet &&) = default;
  constexpr Error operator()(const dummy &) { return Error::none; }
};

template <typename T> struct DefaultGet {
  const T *value_;
  constexpr DefaultGet(const T &v) : value_(&v) {}
  constexpr DefaultGet(const DefaultGet &) = default;
  constexpr DefaultGet(DefaultGet &&) = default;
  constexpr DefaultGet &operator=(const DefaultGet &) = default;
  constexpr DefaultGet &operator=(DefaultGet &&) = default;

  constexpr Error operator()(T &t) {
    if (value_ == nullptr)
      return Error::cant_read_param;
    t = *value_;
    return Error::none;
  }
};

template <typename T> struct DefaultSet {
  T *value_;
  constexpr DefaultSet(T &v) : value_(&v) {}
  constexpr DefaultSet(const DefaultSet &) = default;
  constexpr DefaultSet(DefaultSet &&) = default;
  constexpr DefaultSet &operator=(const DefaultSet &) = default;
  constexpr DefaultSet &operator=(DefaultSet &&) = default;

  constexpr Error operator()(const T &t) {
    if (value_ == nullptr)
      return Error::cant_set_param;
    *value_ = t;
    return Error::none;
  }
};

template <typename T, typename MemberPtr> struct MemDataGet {
  const T &value_;
  MemberPtr member;
  constexpr Error operator()(mem_data_type<MemberPtr> &t) {
    t = (value_.*member);
    return Error::none;
  }
};

template <typename T, typename MemberPtr> struct MemDataSet {
  T &value_;
  MemberPtr member;
  constexpr Error operator()(const mem_data_type<MemberPtr> &t) {
    value_.*member = t;
    return Error::none;
  }
};
template <typename T, typename MemberPtr>
struct MemDataSet<const T, MemberPtr> {
  constexpr Error operator()(const mem_data_type<MemberPtr> &) {
    return Error::cant_set_param;
  }
};
// param("[any.path.]name", global_ref, [get, [set]]):
// param("[any.path.]obj.name", obj, mem_ptr);
// param("obj.name", obj, mem_ptr);
/**
 * @brief
 *
 * @tparam CmdName the name of the parameter
 * @tparam Description the description
 * @tparam Help the help string
 * @tparam T the value type of the parameter
 * @tparam Get the callable thats retrieves a T's value
 * @tparam Set the callable thats sets a T's value
 * @tparam Parse the callable that parses a T from a ByteView
 * @tparam Format the callable that formats a T into a ByteView
 * @tparam SubCommands further Param or Function sub commands
 */
template <Name CmdName, Name Description, Name Help, Getter Get, Setter Set,
          parse::Parser Parse, format::Formatter Format, Command... SubCommands>
class Param : public CommandBase<Param<CmdName, Description, Help, Get, Set,
                                       Parse, Format, SubCommands...>,
                                 CmdName, Description, Help, SubCommands...> {
  using Base = CommandBase<Param<CmdName, Description, Help, Get, Set, Parse,
                                 Format, SubCommands...>,
                           CmdName, Description, Help, SubCommands...>;
  using value_type = typename getter_value_type<Get>::type;
  static_assert(
      std::is_same_v<value_type, typename setter_value_type<Set>::type>,
      "Get and Set must get/set a value of the same type");
  static_assert(std::is_same_v<value_type, parse::value_type_t<Parse>>,
                "Parse and Get/Set must have the same value type");
  static_assert(
      std::is_same_v<value_type,
                     typename format::formatter_value_type<Format>::type>,
      "Format and Get/Set must have the same value type");

public:
  // using Base::description;
  // using Base::name;
  // using Base::type;

  constexpr Param(const Param &) = default;
  constexpr Param(Param &&) = default;
  constexpr Param &operator=(const Param &) = default;
  constexpr Param &operator=(Param &&) = default;

  template <Getter Get_, Setter Set_, parse::Parser Parse_,
            format::Formatter Format_, Command... SubCommands_>
  constexpr Param(CmdName, Description, Help, Get_ &&get, Set_ &&set,
                  Parse_ &&parse, Format_ &&format, SubCommands_ &&...cmds)
      : Base(std::forward<SubCommands_>(cmds)...),
        get_(std::forward<Get_>(get)), set_(std::forward<Set_>(set)),
        parse_(std::forward<Parse_>(parse)),
        format_(std::forward<Format_>(format)) {}

  template <Getter Get_, Setter Set_, parse::Parser Parse_,
            format::Formatter Format_>
  constexpr Param(CmdName, Description, Help, Get_ &&get, Set_ &&set,
                  Parse_ &&parse, Format_ &&format,
                  std::tuple<SubCommands...> &&cmds)
      : Base(std::move(cmds)), get_(std::forward<Get_>(get)),
        set_(std::forward<Set_>(set)), parse_(std::forward<Parse_>(parse)),
        format_(std::forward<Format_>(format)) {}
  template <Getter Get_, Setter Set_, parse::Parser Parse_,
            format::Formatter Format_>
  constexpr Param(CmdName, Description, Help, Get_ &&get, Set_ &&set,
                  Parse_ &&parse, Format_ &&format,
                  const std::tuple<SubCommands...> &cmds)
      : Base(cmds), get_(std::forward<Get_>(get)),
        set_(std::forward<Set_>(set)), parse_(std::forward<Parse_>(parse)),
        format_(std::forward<Format_>(format)) {}

  // Param(Name /*name*/, const T &value, Get getter, Set setter);

  Error execute(ExecType type, const ArgVector &args, std::span<uint8_t> &out) {
    switch (type) {
    case ExecType::set:
      return set_value(args);
    case ExecType::get:
      if (args.size() != 0)
        return Error::too_many_argments;
      return get_value(out);
      break;
    default:
      return Error::invalid_cmd;
    }
    return Error::unimplemented;
  }

private:
  Error set_value(const ArgVector &args) {
    if (args.size() == 0)
      return Error::too_few_arguments;
    auto parse_result = Parse::operator()(args[0]);
    if (not parse_result)
      return parse_result.error;
    return static_cast<Set &>(*this)(parse_result.value);
  }

  Error get_value(std::span<uint8_t> &out) {
    value_type t{};
    static_cast<Get &>(*this)(t);
    auto res = Format::operator()(out, t);
    if (res.error != Error::none)
      return res.error;
    out = out.subspan(res.size_written);
    return Error::none;
  }

  [[no_unique_address]] Get get_;
  [[no_unique_address]] Set set_;
  [[no_unique_address]] Parse parse_;
  [[no_unique_address]] Format format_;
};

template <Name CmdName, Name Description, Name Help, Getter Get, Setter Set,
          parse::Parser Parse, format::Formatter Format, Command... SubCommands>
Param(CmdName, Description, Help, Get &&get, Set &&set, Parse &&parse,
      Format &&format, SubCommands &&...cmds)
    -> Param<std::remove_cvref_t<CmdName>, std::remove_cvref_t<Description>,
             std::remove_cvref_t<Help>, std::remove_cvref_t<Get>,
             std::remove_cvref_t<Set>, std::remove_cvref_t<Parse>,
             std::remove_cvref_t<Format>, std::remove_cvref_t<SubCommands>...>;

template <Name CmdName, Name Description, Name Help, Getter Get, Setter Set,
          parse::Parser Parse, format::Formatter Format, Command... SubCommands>
Param(CmdName, Description, Help, Get &&get, Set &&set, Parse &&parse,
      Format &&format, std::tuple<SubCommands...> &&cmds)
    -> Param<std::remove_cvref_t<CmdName>, std::remove_cvref_t<Description>,
             std::remove_cvref_t<Help>, std::remove_cvref_t<Get>,
             std::remove_cvref_t<Set>, std::remove_cvref_t<Parse>,
             std::remove_cvref_t<Format>, std::remove_cvref_t<SubCommands>...>;

/**
 * @brief
 *
 * @param cmds
 */
template <Name CmdName, Command... SubCommands>
constexpr auto param(CmdName, SubCommands &&...cmds) {
  {
    return Param{CmdName{},
                 NoDescription{},
                 NoHelp{},
                 NullGet{},
                 NullSet{},
                 parse::NullParse{},
                 format::NullFormat{},
                 std::forward<SubCommands>(cmds)...};
  }
}

/**
 * @brief
 *
 * @tparam T
 * @param t
 * @param cmds
 * @return
 */
template <Name CmdName, class T, Command... SubCommands>
constexpr auto param(CmdName, T &t, SubCommands &&...cmds) {
  {
    return Param{CmdName{},
                 NoDescription{},
                 cli::ctti::name<std::remove_cvref_t<T>>(),
                 DefaultGet<T>{t},
                 DefaultSet<T>{t},
                 parse::DefaultParse<T>(),
                 format::DefaultFormat<T>(),
                 std::forward<SubCommands>(cmds)...};
  }
}

template <Name CmdName, Name Description, Name Help, Getter Get, Setter Set,
          parse::Parser Parse, format::Formatter Format, Command... SubCommands>
auto param(CmdName, Description, Help, Get &&get, Set &&set, Parse &&parse,
           Format &&format, SubCommands &&...cmds) {
  return Param{CmdName{},
               Description{},
               Help{},
               std::forward<Get>(get),
               std::forward<Set>(set),
               std::forward<Parse>(parse),
               std::forward<Format>(parse),
               std::forward<SubCommands>(cmds)...};
}

template <Name CmdName, Name Description, Name Help, class MemberPointer,
          parse::ParserOf<mem_data_type<MemberPointer>> Parse,
          format::FormatterOf<mem_data_type<MemberPointer>> Format,
          Command... SubCommands>
struct MemberData {
  MemberPointer f;
  Parse parse;
  Format format;
  std::tuple<SubCommands...> subcommands;

  template <parse::ParserOf<mem_data_type<MemberPointer>> Parse_,
            format::FormatterOf<mem_data_type<MemberPointer>> Format_,
            Command... SubCommands_>
  constexpr MemberData(CmdName, Description, Help, MemberPointer f,
                       Parse_ parse, Format_ format, SubCommands_ &&...cmds)
      : f(f), parse(std::forward<Parse_>(parse)),
        format(std::forward<Format>(format)),
        subcommands(std::forward<SubCommands>(cmds)...) {}
};

template <Name CmdName, Name Description, Name Help, class MemberPointer,
          parse::ParserOf<mem_data_type<MemberPointer>> Parse,
          format::FormatterOf<mem_data_type<MemberPointer>> Format>
struct MemberData<CmdName, Description, Help, MemberPointer, Parse, Format> {
  MemberPointer f;
  Parse parse;
  Format format;

  template <parse::ParserOf<mem_data_type<MemberPointer>> Parse_,
            format::FormatterOf<mem_data_type<MemberPointer>> Format_>
  constexpr MemberData(CmdName, Description, Help, MemberPointer f,
                       Parse_ parse, Format_ format)
      : f(f), parse(std::forward<Parse_>(parse)),
        format(std::forward<Format>(format)) {}
};

template <Name CmdName, Name Description, Name Help, class MemberPointer,
          parse::ParserOf<mem_data_type<MemberPointer>> Parse,
          format::FormatterOf<mem_data_type<MemberPointer>> Format,
          Command... SubCommands>
MemberData(CmdName, Description, Help, MemberPointer, Parse &&, Format &&,
           SubCommands &&...)
    -> MemberData<
        std::remove_cvref_t<CmdName>, std::remove_cvref_t<Description>,
        std::remove_cvref_t<Help>, MemberPointer, std::remove_cvref_t<Parse>,
        std::remove_cvref_t<Format>, std::remove_cvref_t<SubCommands>...>;

template <Name CmdName, class MemberPointer, Command... SubCommands>
constexpr auto mem_data(CmdName, MemberPointer f, SubCommands &&...cmds) {
  return MemberData{CmdName{},
                    NoDescription{},
                    cli::ctti::name<mem_data_type<MemberPointer>>(),
                    f,
                    parse::DefaultParse<mem_data_type<MemberPointer>>{},
                    format::DefaultFormat<mem_data_type<MemberPointer>>{},
                    std::forward<SubCommands>(cmds)...};
}
namespace dtl {
template <class T, Name CmdName, Name Description, Name Help,
          class MemberPointer,
          parse::ParserOf<mem_data_type<MemberPointer>> Parse,
          format::FormatterOf<mem_data_type<MemberPointer>> Format,
          Command... SubCommands>
constexpr auto to_cmd(T &obj,
                      MemberData<CmdName, Description, Help, MemberPointer,
                                 Parse, Format, SubCommands...>
                          member_data) {
  if constexpr (sizeof...(SubCommands) > 0)
    return Param<CmdName, Description, Help, MemDataGet<T, MemberPointer>,
                 MemDataSet<T, MemberPointer>, Parse, Format, SubCommands...>{
        CmdName{},
        Description{},
        Help{},
        MemDataGet<T, MemberPointer>{obj, member_data.f},
        MemDataSet<T, MemberPointer>{obj, member_data.f},
        std::move(member_data.parse),
        std::move(member_data.format),
        std::move(member_data.subcommands)};
  else
    return Param<CmdName, Description, Help, MemDataGet<T, MemberPointer>,
                 MemDataSet<T, MemberPointer>, Parse, Format, SubCommands...>{
        CmdName{},
        Description{},
        Help{},
        MemDataGet<T, MemberPointer>{obj, member_data.f},
        MemDataSet<T, MemberPointer>{obj, member_data.f},
        std::move(member_data.parse),
        std::move(member_data.format)};
}

template <class T, Name CmdName, Name Description, Name Help,
          class MemberPointer,
          parse::ParserOf<mem_data_type<MemberPointer>> Parse,
          format::FormatterOf<mem_data_type<MemberPointer>> Format,
          Command... SubCommands>
constexpr auto to_cmd(const T &obj,
                      MemberData<CmdName, Description, Help, MemberPointer,
                                 Parse, Format, SubCommands...>
                          member_data) {
  if constexpr (sizeof...(SubCommands) > 0)
    return Param<CmdName, Description, Help, MemDataGet<const T, MemberPointer>,
                 MemDataSet<const T, MemberPointer>, Parse, Format,
                 SubCommands...>{
        CmdName{},
        Description{},
        Help{},
        MemDataGet<T, MemberPointer>{obj, member_data.f},
        MemDataSet<T, MemberPointer>{obj, member_data.f},
        std::move(member_data.parse),
        std::move(member_data.format),
        std::move(member_data.subcommands)};
  else
    return Param<CmdName, Description, Help, MemDataGet<const T, MemberPointer>,
                 MemDataSet<const T, MemberPointer>, Parse, Format,
                 SubCommands...>{
        CmdName{},
        Description{},
        Help{},
        MemDataGet<T, MemberPointer>{obj, member_data.f},
        MemDataSet<T, MemberPointer>{obj, member_data.f},
        std::move(member_data.parse),
        std::move(member_data.format)};
}

template <class T, class CommandOrMemberDataOrMemberFunction>
constexpr auto transform(T &obj, CommandOrMemberDataOrMemberFunction &&mem) {
  if constexpr (Command<
                    std::remove_cvref_t<CommandOrMemberDataOrMemberFunction>>) {
    return mem;
  } else {
    using dtl::to_cmd;
    using funcs::dtl::to_cmd;
    return to_cmd(obj, std::forward<CommandOrMemberDataOrMemberFunction>(mem));
  }
}
} // namespace dtl

template <Name CmdName, class T, class... CommandOrMemberDataOrMemberFunction>
constexpr auto obj(CmdName, T &obj,
                   CommandOrMemberDataOrMemberFunction &&...m) {
  return Param{
      CmdName{},
      NoDescription{},
      cli::ctti::name<std::remove_cvref_t<T>>(),
      DefaultGet<T>{obj},
      DefaultSet<T>{obj},
      parse::DefaultParse<T>{},
      format::DefaultFormat<T>{},
      dtl::transform(obj,
                     std::forward<CommandOrMemberDataOrMemberFunction>(m))...};
}

template <auto &Obj, class... CommandOrMemberDataOrMemberFunction>
constexpr auto obj(CommandOrMemberDataOrMemberFunction &&...m) {
  using T = std::remove_cvref_t<decltype(Obj)>;
  return Param{
      ctti::object_name<Obj>(),
      NoDescription{},
      cli::ctti::name<T>(),
      DefaultGet<T>{Obj},
      DefaultSet<T>{Obj},
      parse::DefaultParse<T>{},
      format::DefaultFormat<T>{},
      dtl::transform(Obj,
                     std::forward<CommandOrMemberDataOrMemberFunction>(m))...};
}

template <class T, class... CommandOrMemberDataOrMemberFunction>
constexpr auto obj(T &obj, CommandOrMemberDataOrMemberFunction &&...m) {
  return Param{
      cli::to_lower(cli::ctti::name<std::remove_cvref_t<T>>()),
      NoDescription{},
      cli::ctti::name<std::remove_cvref_t<T>>(),
      DefaultGet<T>{obj},
      DefaultSet<T>{obj},
      parse::DefaultParse<T>{},
      format::DefaultFormat<T>{},
      dtl::transform(obj,
                     std::forward<CommandOrMemberDataOrMemberFunction>(m))...};
}

} // namespace cli::params
#endif
