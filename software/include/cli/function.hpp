#ifndef CLI_FUNCTION_HPP
#define CLI_FUNCTION_HPP

#include "cli/ctti.hpp"
#include "cli/enums.hpp"
#include "cli/format.hpp"
#include "cli/parse.hpp"
#include "cli/type_list.hpp"
#include "cli/util.hpp"
#include "cli/validator.hpp"

#include <concepts>
#include <type_traits>
#include <utility>

namespace cli::funcs {

template <class A>
concept FuncArg = requires(A &&arg) {
  { typename std::remove_cvref_t<A>::name{} } -> Name;
  { typename std::remove_cvref_t<A>::description{} } -> Name;
  { arg.parse } -> parse::Parser;
  { arg.validate } -> validate::Validator;
  // {
  //   std::remove_cvref_t<A>::default_value
  // } /* -> std::same_as<typename std::remove_cvref_t<A>::type> */;
};

inline constexpr struct Deduced {
  constexpr Deduced() = default;
} deduced{};

template <Name ArgName, Name Description, class T, auto DefaultValue,
          parse::Parser Parse, validate::Validator Validate>
struct FunctionArg {
  using name = ArgName;
  using description = Description;
  using type = std::remove_cvref_t<decltype(DefaultValue)>;
  using parser = Parse;
  using validator = Validate;

  template <parse::Parser P, validate::Validator V>
  constexpr FunctionArg(ArgName, Description, identity<T>,
                        constant<DefaultValue>, P &&parse, V &&validate)
      : parse(std::forward<P>(parse)), validate(std::forward<V>(validate)) {}

  template <parse::Parser P, validate::Validator V>
  constexpr FunctionArg(ArgName, Description, constant<DefaultValue>, P &&parse,
                        V &&validate)
      : parse(std::forward<P>(parse)), validate(std::forward<V>(validate)) {}
  template <parse::Parser P, validate::Validator V>
  constexpr FunctionArg(ArgName, Description, P &&parse, V &&validate)
      : parse(std::forward<P>(parse)), validate(std::forward<V>(validate)) {}

  CLI_NO_UNIQUE_ADDRESS Parse parse{};
  CLI_NO_UNIQUE_ADDRESS Validate validate{};
};

template <Name ArgName, Name Description, class T, auto DefaultValue,
          parse::Parser Parse, validate::Validator Validate>
FunctionArg(ArgName, Description, identity<T>, constant<DefaultValue>, Parse &&,
            Validate &&)
    -> FunctionArg<ArgName, Description, T, DefaultValue,
                   std::remove_cvref_t<Parse>, std::remove_cvref_t<Validate>>;

template <Name ArgName, Name Description, auto DefaultValue,
          parse::Parser Parse, validate::Validator Validate>
FunctionArg(ArgName, Description, constant<DefaultValue>, Parse &&, Validate &&)
    -> FunctionArg<ArgName, Description, parse::value_type_t<Parse>,
                   DefaultValue, std::remove_cvref_t<Parse>,
                   std::remove_cvref_t<Validate>>;

template <Name ArgName, Name Description, parse::Parser Parse,
          validate::Validator Validate>
FunctionArg(ArgName, Description, Parse &&, Validate &&)
    -> FunctionArg<ArgName, Description, parse::value_type_t<Parse>,
                   parse::value_type_t<Parse>{}, std::remove_cvref_t<Parse>,
                   std::remove_cvref_t<Validate>>;

template <Name ArgName, Name Description, typename T, parse::Parser Parse,
          validate::Validator Validate>
struct FunctionArgWithoutDefault {
  using name = ArgName;
  using description = Description;
  using type = T;
  using parser = Parse;
  using validator = Validate;

  template <parse::Parser P, validate::Validator V>
  constexpr FunctionArgWithoutDefault(ArgName, Description, identity<T>,
                                      P &&parse, V &&validate)
      : parse(std::forward<P>(parse)), validate(std::forward<V>(validate)) {}

  template <parse::Parser P, validate::Validator V>
  constexpr FunctionArgWithoutDefault(ArgName, Description, P &&parse,
                                      V &&validate)
      : parse(std::forward<P>(parse)), validate(std::forward<V>(validate)) {}

  CLI_NO_UNIQUE_ADDRESS Parse parse{};
  CLI_NO_UNIQUE_ADDRESS Validate validate{};
};

template <Name ArgName, Name Description, typename T, parse::Parser Parse,
          validate::Validator Validate>
FunctionArgWithoutDefault(ArgName, Description, identity<T>, Parse &&,
                          Validate &&)
    -> FunctionArgWithoutDefault<ArgName, Description, T,
                                 std::remove_cvref_t<Parse>,
                                 std::remove_cvref_t<Validate>>;

template <Name ArgName, Name Description, parse::Parser Parse,
          validate::Validator Validate>
FunctionArgWithoutDefault(ArgName, Description, Parse &&, Validate &&)
    -> FunctionArgWithoutDefault<
        ArgName, Description, parse::value_type_t<Parse>,
        std::remove_cvref_t<Parse>, std::remove_cvref_t<Validate>>;

template <Name ArgName, Name Description> struct DeducedArg {
  using name = ArgName;
  using description = Description;
  using type = Deduced;
  using parser = parse::DefaultParse<Deduced>;
  using validator = validate::DefaultValidate<Deduced>;
};

namespace dtl {
template <Callable F, std::size_t I, Name N, Name D, class T, parse::Parser P,
          validate::Validator V>
constexpr auto deduce_arg(const FunctionArgWithoutDefault<N, D, T, P, V> &arg) {
  using args = typename function_traits<F>::arguments;
  using arg_type = std::remove_cvref_t<type_list::type_at_t<I, args>>;
  static_assert(parse::ParserOf<P, T>);
  static_assert(validate::ValidatorOf<V, T>);
  static_assert(
      std::same_as<arg_type, T>,
      "the I-th arg's explicitly set type does not match F's I-th argument");
  return arg;
}

template <Callable F, std::size_t I, Name N, Name D, class T, auto DV,
          parse::Parser P, validate::Validator V>
constexpr auto deduce_arg(const FunctionArg<N, D, T, DV, P, V> &arg) {
  using args = typename function_traits<F>::arguments;
  using arg_type = std::remove_cvref_t<type_list::type_at_t<I, args>>;
  static_assert(parse::ParserOf<P, T>);
  static_assert(validate::ValidatorOf<V, T>);
  static_assert(std::constructible_from<T, decltype(DV)>);
  static_assert(
      std::same_as<arg_type, T>,
      "the I-th arg's explicitly set type does not match F's I-th argument");
  return arg;
}

template <Callable F, std::size_t I, Name N, Name D>
constexpr auto deduce_arg(const DeducedArg<N, D> &arg) {
  using args = typename function_traits<F>::arguments;
  using type = std::remove_cvref_t<type_list::type_at_t<I, args>>;
  if constexpr (std::is_same_v<D, string_constant<>> or
                std::is_same_v<D, NoDescription>)
    return FunctionArgWithoutDefault{
        N{}, cli::ctti::name<type>(), identity<type>{},
        parse::DefaultParse<type>{}, validate::DefaultValidate<type>{}};
  else
    return FunctionArgWithoutDefault{N{}, D{}, identity<type>{},
                                     parse::DefaultParse<type>{},
                                     validate::DefaultValidate<type>{}};
}

template <Callable F, class... Args>
constexpr auto deduce_args(const Args &...args) {
  return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    return std::tuple(deduce_arg<F, Is>(args)...);
  }(std::make_index_sequence<sizeof...(Args)>());
}

template <Name ArgName, Name Description, class T, parse::Parser Parse,
          validate::Validator Validate>
constexpr auto
pretty_arg_name(const FunctionArgWithoutDefault<ArgName, Description, T, Parse,
                                                Validate> &) {
  return ArgName{} + ": "_sc + ctti::name<T>();
}

template <Name ArgName, Name Description, class T, auto DefaultValue,
          parse::Parser Parse, validate::Validator Validate>
constexpr auto
pretty_arg_name(const FunctionArg<ArgName, Description, T, DefaultValue, Parse,
                                  Validate> &) {
  return ArgName{} + ": "_sc + ctti::name<T>() + "?"_sc;
}

template <FuncArg A, FuncArg... As>
constexpr auto make_pretty_signature_name(const A &arg, const As &...args) {
  if constexpr (sizeof...(As) == 0)
    return pretty_arg_name(arg);
  else
    return pretty_arg_name(arg) + ", "_sc + make_pretty_signature_name(args...);
}

template <Callable F, FuncArg... Args>
constexpr auto pretty_signature_name(const Args &...args) {
  return "("_sc + make_pretty_signature_name(args...) + ")->"_sc +
         ctti::name<typename function_traits<F>::return_type>();
}

template <Callable F, FuncArg... Args>
constexpr auto pretty_signature_name(const std::tuple<Args...> &args) {
  return []<std::size_t... Is>(std::index_sequence<Is...>,
                               const std::tuple<Args...> &args) {
    return "("_sc + make_pretty_signature_name(std::get<Is>(args)...) +
           ")->"_sc + ctti::name<typename function_traits<F>::return_type>();
  }(std::make_index_sequence<sizeof...(Args)>(), args);
}

template <Callable F> constexpr auto pretty_signature_name() {
  return "()->"_sc + ctti::name<typename function_traits<F>::return_type>();
}

template <Name ArgName, Name Description, class T, auto DefaultValue,
          parse::Parser Parse, validate::Validator Validate>
constexpr auto
parse_field_from_arg(const FunctionArg<ArgName, Description, T, DefaultValue,
                                       Parse, Validate> &arg) {
  return parse::Field<ArgName, DefaultValue, Parse>{DefaultValue, arg.parse};
}

template <Name ArgName, Name Description, typename T, parse::Parser Parse,
          validate::Validator Validate>
constexpr auto
parse_field_from_arg(const FunctionArgWithoutDefault<ArgName, Description, T,
                                                     Parse, Validate> &arg) {
  return parse::FieldWithOutDefault<ArgName, T, Parse>{T{}, arg.parse};
}

template <class... Args>
constexpr auto parse_field_from_args(const std::tuple<Args...> &args) {
  return [&args]<std::size_t... Is>(std::index_sequence<Is...>) {
    return std::tuple(parse_field_from_arg(std::get<Is>(args))...);
  }(std::make_index_sequence<sizeof...(Args)>{});
}
} // namespace dtl

template <auto DefaultValue, Name ArgName, Name Description,
          parse::Parser Parse, validate::Validator Validate>
constexpr auto arg(ArgName, Description, Parse &&parse, Validate &&validate) {
  return FunctionArg{ArgName{}, Description{}, constant<DefaultValue>{},
                     std::forward<Parse>(parse),
                     std::forward<Validate>(validate)};
}

template <class T, auto DefaultValue, Name ArgName, Name Description>
constexpr auto arg(ArgName, Description) {
  return FunctionArg{ArgName{},
                     Description{},
                     identity<T>{},
                     constant<DefaultValue>{},
                     parse::DefaultParse<T>{},
                     validate::DefaultValidate<T>{}};
}

template <class T, auto DefaultValue, Name ArgName>
constexpr auto arg(ArgName) {
  return FunctionArg{ArgName{},
                     NoDescription{},
                     identity<T>{},
                     constant<DefaultValue>{},
                     parse::DefaultParse<T>{},
                     validate::DefaultValidate<T>{}};
}

template <class T, Name ArgName, Name Description, parse::ParserOf<T> Parse,
          validate::ValidatorOf<T> Validate>
constexpr auto arg(ArgName, Description, Parse &&parse, Validate &&validate) {
  return FunctionArgWithoutDefault{ArgName{}, Description{}, identity<T>{},
                                   std::forward<Parse>(parse),
                                   std::forward<Validate>(validate)};
}

template <Name ArgName, Name Description>
constexpr auto arg(ArgName, Description) {
  return DeducedArg<ArgName, Description>{};
}

template <Name ArgName> constexpr auto arg(ArgName) {
  return DeducedArg<ArgName, NoDescription>{};
}

template <cli::StringLiteral S> constexpr auto operator""_arg() {
  return arg([&]<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<S.s[Is]...>{};
  }(std::make_index_sequence<S.size()>()));
}

template <Callable F, FuncArg... Args>
constexpr Error parse_and_call(F &&f, ByteView args, std::span<uint8_t> &out) {
  return Error::unimplemented;
}

template <Name FuncName, Name Description, Name Type, Callable F,
          FuncArg... Args>
class Function
    : public CommandBase<Function<FuncName, Description, Type, F, Args...>,
                         FuncName, Description, Type> {
  using Base = CommandBase<Function<FuncName, Description, Type, F, Args...>,
                           FuncName, Description, Type>;
  using traits = function_traits<F>;
  using arguments = typename traits::arguments;

  template <class... Fields>
  using PartialParser = parse::FieldGroup<'=', ',', ' ', ' ', Fields...>;
  using Parser = type_list::apply_t<PartialParser,
                                    decltype(dtl::parse_field_from_args(
                                        std::declval<std::tuple<Args...>>()))>;

public:
  using Base::description;
  using Base::name;
  using Base::type;
  using sub_command_list = TypeList<>;

  using signature = typename traits::signature_type;

  template <Callable Func, FuncArg... A>
  constexpr Function(FuncName, Description, Type, Func &&function, A &&...args)
      : func_(std::forward<Func>(function)), args(std::forward<A>(args)...) {}

  template <Callable Func>
  constexpr Function(FuncName, Description, Type, Func &&function,
                     const std::tuple<Args...> &args)
      : func_(std::forward<Func>(function)), args(args) {}

  template <Callable Func>
  constexpr Function(FuncName, Description, Type, Func &&function,
                     std::tuple<Args...> &&args)
      : func_(std::forward<Func>(function)), args(std::move(args)) {}

  Error execute(ExecType type, ArgVector args,
                [[maybe_unused]] std::span<uint8_t> &out) {
    using Ret = typename traits::return_type;

    if (type != ExecType::call)
      return Error::invalid_cmd;

    const Parser parse{};

    auto res = parse(args);
    if (not res)
      return res.error;
    return [&tuple = res.value, &out,
            this]<std::size_t... Is>(std::index_sequence<Is...>) {
      const Error err = validate(tuple, std::index_sequence<Is...>{});
      if (err != Error::none)
        return err;

      if constexpr (std::is_same_v<void, Ret>) {
        static_cast<void>(out);
        func_(std::get<Is>(tuple).value...);
        return Error::none;
      } else {
        auto res = func_(std::get<Is>(tuple).value...);
        format::DefaultFormat<Ret> format;
        auto fmt_result = format(out, res);
        out = out.subspan(0, fmt_result.size_written);
        return fmt_result.error;
      }
    }(std::make_index_sequence<sizeof...(Args)>());
    return Error::unimplemented;
  }

  template <std::size_t I, std::size_t... Is>
  static constexpr cli::Error validate(const auto &tuple,
                                       std::index_sequence<I, Is...>) {
    auto err = typename type_list::type_at_t<I, TypeList<Args...>>::validator{}(
        std::get<I>(tuple).value);
    if constexpr (sizeof...(Is) == 0)
      return err;
    else {
      if (err != Error::none)
        return err;
      return validate(tuple, std::index_sequence<Is...>{});
    }
  }
  ByteView get_help() {}

private:
  F func_{};
  std::tuple<Args...> args;
};

template <Name FuncName, Name Description, Name Type, Callable F>
class Function<FuncName, Description, Type, F>
    : public CommandBase<Function<FuncName, Description, Type, F>, FuncName,
                         Description, Type> {
  using Base = CommandBase<Function<FuncName, Description, Type, F>, FuncName,
                           Description, Type>;
  using traits = function_traits<F>;
  using arguments = typename traits::arguments;

public:
  using Base::description;
  using Base::name;
  using Base::type;
  using sub_command_list = TypeList<>;

  using signature = typename traits::signature_type;

  template <Callable Func>
  constexpr Function(FuncName, Description, Type, Func &&function)
      : func_(std::forward<Func>(function)) {}

  Error execute(ExecType type, ArgVector args,
                [[maybe_unused]] std::span<uint8_t> &out) {
    using Ret = typename traits::return_type;

    if (type != ExecType::call)
      return Error::invalid_cmd;

    if constexpr (std::is_same_v<void, Ret>) {
      func_();
      return Error::none;
    } else {
      auto res = format_(out, func_());
      return res.error;
    }
    return Error::unimplemented;
  }
  ByteView get_help() {}

private:
  F func_{};
};

// template <Name FuncName, Name Description, Name Help, Callable F>
// Function(FuncName, Description, Help, F &&)
//     -> Function<FuncName, Description, Help, std::remove_cvref_t<F>>;

template <Name FuncName, Name Description, Name Type, Callable F,
          FuncArg... Args>
Function(FuncName, Description, Type, F &&, Args &&...)
    -> Function<FuncName, Description, Type, std::remove_cvref_t<F>,
                std::remove_cvref_t<Args>...>;

template <Name FuncName, Name Description, Name Type, Callable F,
          FuncArg... Args>
Function(FuncName, Description, Type, F &&, const std::tuple<Args...> &)
    -> Function<FuncName, Description, Type, std::remove_cvref_t<F>, Args...>;

template <Name FuncName, Name Description, Name Type, Callable F,
          FuncArg... Args>
Function(FuncName, Description, Type, F &&, std::tuple<Args...> &&)
    -> Function<FuncName, Description, Type, std::remove_cvref_t<F>, Args...>;
/**
 * @brief
 *
 * @param f
 * @param args
 * @return
 */
template <Name FuncName, Name Description, Name Help, Callable F, class... Args>
constexpr auto func(FuncName, Description, Help, F &&f, Args &&...args) {
  return Function<FuncName, Description, Help, std::decay_t<F>,
                  std::decay_t<Args>...>{
      FuncName{}, Description{}, Help{}, std::forward<F>(f),
      dtl::deduce_args<F>(std::forward<Args>(args)...)};
}

template <Name FuncName, Callable F, class... Args>
constexpr auto func(FuncName, F &&f, Args &&...args) {
  static_assert(
      type_list::list_size_v<typename function_traits<F>::arguments> ==
          sizeof...(Args),
      "All arguments of F must be named");
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<F>(std::forward<Args>(args)...);
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<F>(deduced), std::forward<F>(f),
                    deduced};
  } else {
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<F>(), std::forward<F>(f)};
  }
}

template <Callable F, class... Args>
  requires(not std::is_pointer_v<std::decay_t<F>>)
constexpr auto func(F &&f, Args &&...args) {
  static_assert(
      type_list::list_size_v<typename function_traits<F>::arguments> ==
          sizeof...(Args),
      "All arguments of F must be named");
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<F>(std::forward<Args>(args)...);
    return Function{to_lower(ctti::name<std::remove_cvref_t<F>>()),
                    NoDescription{}, dtl::pretty_signature_name<F>(deduced),
                    std::forward<F>(f), deduced};
  } else {
    return Function{to_lower(ctti::name<std::remove_cvref_t<F>>()),
                    NoDescription{}, dtl::pretty_signature_name<F>(),
                    std::forward<F>(f)};
  }
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          class... Args>
constexpr auto func(FuncName, T &t, Ret (T::*mem_fun)(MemFunArgs...),
                    Args &&...args) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  using Binder = MemFunBinder<T, Ret (T::*)(MemFunArgs...)>;
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<Binder>(std::forward<Args>(args)...);
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(deduced),
                    Binder{t, mem_fun}, deduced};
  } else {
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(),
                    Binder{t, mem_fun}};
  }
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          class... Args>
constexpr auto func(FuncName, T &t, Ret (T::*mem_fun)(MemFunArgs...) noexcept,
                    Args &&...args) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  using Binder = MemFunBinder<T, Ret (T::*)(MemFunArgs...) noexcept>;
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<Binder>(std::forward<Args>(args)...);
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(deduced),
                    Binder{t, mem_fun}, deduced};
  } else {
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(),
                    Binder{t, mem_fun}};
  }
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          class... Args>
constexpr auto func(FuncName, const T &t,
                    Ret (T::*mem_fun)(MemFunArgs...) const, Args &&...args) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  using Binder = MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const>;
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<Binder>(std::forward<Args>(args)...);
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(deduced),
                    Binder{t, mem_fun}, deduced};
  } else {
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(),
                    Binder{t, mem_fun}};
  }
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          class... Args>
constexpr auto func(FuncName, const T &t,
                    Ret (T::*mem_fun)(MemFunArgs...) const noexcept,
                    Args &&...args) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  using Binder =
      MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const noexcept>;
  if constexpr (sizeof...(Args) > 0) {
    auto deduced = dtl::deduce_args<Binder>(std::forward<Args>(args)...);
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(deduced),
                    Binder{t, mem_fun}, deduced};
  } else {
    return Function{FuncName{}, NoDescription{},
                    dtl::pretty_signature_name<decltype(mem_fun)>(),
                    Binder{t, mem_fun}};
  }
}

template <Name FuncName, Name Description, Name Help, class Function,
          class... Args>
struct MemberFunction {
  using arguments = TypeList<Args...>;
  static_assert(std::is_member_function_pointer_v<Function>,
                "A MemberFunctions Function template argument must be a "
                "pointer to member function");
  Function f;
  std::tuple<Args...> args;

  constexpr MemberFunction(FuncName, Description, Help, Function mem_fun_ptr,
                           const Args &...args) noexcept
      : f(mem_fun_ptr), args(args...) {}

  constexpr MemberFunction(FuncName, Description, Help, Function mem_fun_ptr,
                           const std::tuple<Args...> &args) noexcept
      : f(mem_fun_ptr), args(args) {}
};

template <Name FuncName, Name Description, Name Help, class Function>
struct MemberFunction<FuncName, Description, Help, Function> {
  using arguments = TypeList<>;
  static_assert(std::is_member_function_pointer_v<Function>,
                "A MemberFunctions Function template argument must be a "
                "pointer to member function");
  Function f;

  constexpr MemberFunction(FuncName, Description, Help,
                           Function mem_fun_ptr) noexcept
      : f(mem_fun_ptr) {}
};

template <Name FuncName, Name Description, Name Help, class Function,
          class... Args>
MemberFunction(FuncName &&, Description &&, Help &&, Function &&, Args &&...)
    -> MemberFunction<std::remove_cvref_t<FuncName>,
                      std::remove_cvref_t<Description>,
                      std::remove_cvref_t<Help>, std::remove_cvref_t<Function>,
                      std::remove_cvref_t<Args>...>;
template <Name FuncName, Name Description, Name Help, class Function,
          FuncArg... Args>
MemberFunction(FuncName &&, Description &&, Help &&, Function &&,
               const std::tuple<Args...> &)
    -> MemberFunction<std::remove_cvref_t<FuncName>,
                      std::remove_cvref_t<Description>,
                      std::remove_cvref_t<Help>, std::remove_cvref_t<Function>,
                      std::remove_cvref_t<Args>...>;

template <Name FuncName, Name Description, Name Help, class Function>
MemberFunction(FuncName &&, Description &&, Help &&, Function &&)
    -> MemberFunction<std::remove_cvref_t<FuncName>,
                      std::remove_cvref_t<Description>,
                      std::remove_cvref_t<Help>, std::remove_cvref_t<Function>>;

template <Name FuncName, class MemberFunctionPointer, class... Args>
  requires std::is_member_function_pointer_v<MemberFunctionPointer>
constexpr auto mem_fun(FuncName, MemberFunctionPointer mem_fun,
                       Args &&...args) {
  if constexpr (sizeof...(Args) > 0) {
    constexpr auto deduced_args =
        dtl::deduce_args<decltype(mem_fun)>(std::forward<Args>(args)...);
    return MemberFunction{
        FuncName{}, NoDescription{},
        dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
        deduced_args};
  } else {
    return MemberFunction{FuncName{}, NoDescription{},
                          dtl::pretty_signature_name<decltype(mem_fun)>(),
                          mem_fun};
  }
}

template <auto MemberFunctionPointer, class... Args>
  requires std::is_member_function_pointer_v<decltype(MemberFunctionPointer)>
constexpr auto mem_fun(Args &&...args) {
  if constexpr (sizeof...(Args) > 0) {
    constexpr auto deduced_args =
        dtl::deduce_args<decltype(MemberFunctionPointer)>(
            std::forward<Args>(args)...);
    return MemberFunction{
        ctti::value_name<MemberFunctionPointer>(), NoDescription{},
        dtl::pretty_signature_name<decltype(MemberFunctionPointer)>(
            deduced_args),
        MemberFunctionPointer, deduced_args};
  } else {
    return MemberFunction{
        ctti::value_name<MemberFunctionPointer>(), NoDescription{},
        dtl::pretty_signature_name<decltype(MemberFunctionPointer)>(),
        MemberFunctionPointer};
  }
}

// template <Name FuncName, class T, typename Ret, class... MemFunArgs,
//           FuncArg... Args>
// constexpr auto mem_fun(FuncName, Ret (T::*mem_fun)(MemFunArgs...),
//                        Args &&...args) {
//   constexpr auto deduced_args = dtl::deduce_args<decltype(mem_fun)>(
//       TypeList<std::remove_cvref_t<Args>...>{});
//   return MemberFunction{
//       FuncName{}, NoDescription{},
//       dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
//       deduced_args};
// }
//
// template <Name FuncName, class T, typename Ret, class... MemFunArgs,
//           FuncArg... Args>
// constexpr auto mem_fun(FuncName, Ret (T::*mem_fun)(MemFunArgs...) const,
//                        Args &&...args) {
//   constexpr auto deduced_args = dtl::deduce_args<decltype(mem_fun)>(
//       TypeList<std::remove_cvref_t<Args>...>{});
//   return MemberFunction{
//       FuncName{}, NoDescription{},
//       dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
//       deduced_args};
// }
//
// template <Name FuncName, class T, typename Ret, class... MemFunArgs,
//           FuncArg... Args>
// constexpr auto mem_fun(FuncName, Ret (T::*mem_fun)(MemFunArgs...) noexcept,
//                        Args &&...args) {
//   constexpr auto deduced_args = dtl::deduce_args<decltype(mem_fun)>(
//       TypeList<std::remove_cvref_t<Args>...>{});
//   return MemberFunction{
//       FuncName{}, NoDescription{},
//       dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
//       deduced_args};
// }
//
// template <Name FuncName, class T, typename Ret, class... MemFunArgs,
//           FuncArg... Args>
// constexpr auto mem_fun(FuncName,
//                        Ret (T::*mem_fun)(MemFunArgs...) const noexcept,
//                        Args &&...args) {
//   constexpr auto deduced_args = dtl::deduce_args<decltype(mem_fun)>(
//       TypeList<std::remove_cvref_t<Args>...>{});
//   return MemberFunction{
//       FuncName{}, NoDescription{},
//       dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
//       deduced_args};
// }

namespace dtl {
/**
 * @brief create a Function from an object reference and a MemberFunction
 */
template <class T, Name CmdName, Name Description, Name Help, class F,
          class... Args>
constexpr auto
to_cmd(T &obj, const MemberFunction<CmdName, Description, Help, F, Args...>
                   &member_function) {
  if constexpr (sizeof...(Args) == 0)
    return Function(CmdName{}, Description{}, Help{},
                    MemFunBinder(obj, member_function.f));
  else
    return Function(CmdName{}, Description{}, Help{},
                    MemFunBinder(obj, member_function.f), member_function.args);
}
/**
 * @brief create a Function from an object referenece and a MemberFunction
 */
template <class T, Name CmdName, Name Description, Name Help, class F,
          class... Args>
constexpr auto to_cmd(const T &obj,
                      const MemberFunction<CmdName, Description, Help, F,
                                           Args...> &member_function) {
  if constexpr (sizeof...(Args) == 0)
    return Function(CmdName{}, Description{}, Help{},
                    MemFunBinder(obj, member_function.f));
  else
    return Function(CmdName{}, Description{}, Help{},
                    MemFunBinder(obj, member_function.f), member_function.args);
}
} // namespace dtl
} // namespace cli::funcs
#endif
