#ifndef CLI_FUNCTION_HPP
#define CLI_FUNCTION_HPP

#include "cli/ctti.hpp"
#include "cli/format.hpp"
#include "cli/parse.hpp"
#include "cli/type_list.hpp"
#include "cli/util.hpp"
#include "cli/validator.hpp"
#include <type_traits>
#include <utility>

namespace cli::funcs {

template <class A>
concept FuncArg = requires() {
  { typename std::remove_cvref_t<A>::name{} } -> Name;
  { typename std::remove_cvref_t<A>::description{} } -> Name;
  { typename std::remove_cvref_t<A>::validator{} } -> validate::Validator;
};

struct Deduced {};

template <Name ArgName, Name Description, typename T,
          validate::Validator Validator>
struct FunctionArg {
  using name = ArgName;
  using description = Description;
  using type = T;
  using validator = Validator;
};

namespace dtl {

template <FuncArg Arg> constexpr auto pretty_arg_name() {
  return typename Arg::name{} + ": "_sc + ctti::name<typename Arg::type>();
}
template <FuncArg A, FuncArg... As>
constexpr auto make_pretty_signature_name(TypeList<A, As...>) {
  if constexpr (sizeof...(As) == 0)
    return pretty_arg_name<A>();
  else
    return pretty_arg_name<A>() + ", "_sc +
           make_pretty_signature_name(TypeList<As...>{});
}

template <Callable F, FuncArg... Args>
constexpr auto pretty_signature_name(TypeList<Args...>) {
  return "("_sc + make_pretty_signature_name(TypeList<Args...>{}) + ") -> "_sc +
         ctti::name<typename function_traits<F>::return_type>();
}

template <Callable F> constexpr auto pretty_signature_name(TypeList<>) {
  return "() -> "_sc + ctti::name<typename function_traits<F>::return_type>();
}

template <Callable F, std::size_t I, FuncArg Arg> constexpr auto deduce_arg() {
  using args_ = typename function_traits<F>::arguments;
  using arg_type = std::remove_cvref_t<type_list::type_at_t<I, args_>>;
  using name = typename std::remove_cvref_t<Arg>::name;
  using desc = typename std::remove_cvref_t<Arg>::description;
  using type = typename std::remove_cvref_t<Arg>::type;
  using validator = typename std::remove_cvref_t<Arg>::validator;
  using validator_type = validate::value_type_t<validator>;
  static_assert(
      std::same_as<arg_type, type>,
      "the I-th arg's explicitly set type does not match F's I-th argument");
  if constexpr (std::same_as<validator_type, Deduced>) {
    return FunctionArg<name, desc, type, validate::DefaultValidate<type>>{};
  } else {
    static_assert(validate::ValidatorOf<validator, type>,
                  "the arguments type and its Validator do not match");
    return Arg{};
  }
}

template <Callable F, std::size_t I, FuncArg Arg>
  requires std::same_as<typename std::remove_cvref_t<Arg>::type, Deduced>
constexpr auto deduce_arg() {
  using args = typename function_traits<F>::arguments;
  using type = std::remove_cvref_t<type_list::type_at_t<I, args>>;
  return FunctionArg<typename Arg::name, decltype(cli::ctti::name<type>()),
                     type, validate::DefaultValidate<type>>();
}

template <Callable F, FuncArg... Args>
constexpr auto deduce_args(TypeList<Args...>) {
  return []<std::size_t... Is, FuncArg... A>(std::index_sequence<Is...>,
                                             TypeList<A...>) {
    return TypeList<decltype(deduce_arg<F, Is, A>())...>{};
  }(std::make_index_sequence<sizeof...(Args)>(), TypeList<Args...>{});
}

} // namespace dtl

template <Name ArgName, Name Description, validate::Validator Validator>
constexpr auto arg(ArgName &&, Description &&, Validator &&v) -> FunctionArg<
    std::remove_cvref_t<ArgName>, std::remove_cvref_t<Description>,
    validate::value_type_t<Validator>, std::remove_cvref_t<Validator>> {
  return {};
}

template <Name ArgName>
constexpr auto arg(ArgName &&)
    -> FunctionArg<std::remove_cvref_t<ArgName>, string_constant<>, Deduced,
                   validate::DefaultValidate<Deduced>> {
  return {};
}

template <typename T, Name ArgName> constexpr auto arg(ArgName &&) {
  constexpr auto desc = cli::ctti::name<T>();
  return FunctionArg<std::remove_cvref_t<ArgName>,
                     std::remove_cvref_t<decltype(desc)>, T,
                     validate::DefaultValidate<T>>{};
}

template <cli::StringLiteral S> constexpr auto operator""_a() {
  return arg([&]<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<S.s[Is]...>{};
  }(std::make_index_sequence<S.size()>()));
}
template <Name FuncName, Name Description, Name Type, Callable F,
          FuncArg... Args>
class Function
    : public CommandBase<Function<FuncName, Description, Type, F, Args...>,
                         FuncName, Description, Type> {
  using Base = CommandBase<Function<FuncName, Description, Type, F, Args...>,
                           FuncName, Description, Type>;
  using traits = function_traits<F>;
  using Tuple = type_list::apply_t<std::tuple, typename traits::arguments>;
  using arguments = typename traits::arguments;

public:
  using Base::description;
  using Base::name;
  using Base::type;
  using sub_command_list = TypeList<>;

  using signature = typename traits::signature_type;

  template <Callable Func>
  constexpr Function(FuncName, Description, Type, Func &&function, Args...)
      : func_(std::forward<Func>(function)) {}
  template <Callable Func>
  constexpr Function(FuncName, Description, Type, Func &&function,
                     TypeList<Args...>)
      : func_(std::forward<Func>(function)) {}

  Error execute(ExecType type, const ArgVector &args, std::span<uint8_t> &out) {
    using Ret = typename traits::return_type;
    // TODO: parse args, apply them to func_, and format the result to the
    // output
    if constexpr (type_list::list_size_v<arguments> == 0) {
      if constexpr (std::is_same_v<void, Ret>) {
        func_();
        return Error::none;
      } else if constexpr (std::is_same_v<Error, Ret>) {
        return func_();
      } else {
        format::DefaultFormat<Ret> format;
        auto res = format(out, func_());
        return res.error;
      }
    } else {
      Tuple tuple{};
      auto err = parse::parse_args(tuple, args);
      if (err != Error::none)
        return err;
      if constexpr (std::is_same_v<void, Ret>) {
        cli::apply(tuple, func_);
        return Error::none;
      } else if constexpr (std::is_same_v<Error, Ret>) {
        return cli::apply(tuple, func_);
      } else {
        auto res = cli::apply(tuple, func_);
        format::DefaultFormat<Ret> format;
        auto fmt_result = format(out, res);
        return fmt_result.error;
      }
    }
    return Error::unimplemented;
  }

  ByteView get_help() {}

private:
  F func_{};
};

template <Name FuncName, Name Description, Name Help, Callable F,
          FuncArg... Args>
Function(FuncName, Description, Help, F &&, Args &&...)
    -> Function<std::remove_cvref_t<FuncName>, std::remove_cvref_t<Description>,
                std::remove_cvref_t<Help>, std::remove_cvref_t<F>,
                std::remove_cvref_t<Args>...>;

template <Name FuncName, Name Description, Name Help, Callable F,
          FuncArg... Args>
Function(FuncName, Description, Help, F &&, TypeList<Args...>)
    -> Function<std::remove_cvref_t<FuncName>, std::remove_cvref_t<Description>,
                std::remove_cvref_t<Help>, std::remove_cvref_t<F>,
                std::remove_cvref_t<Args>...>;

/**
 * @brief
 *
 * @param f
 * @param args
 * @return
 */
template <Name FuncName, Name Description, Name Help, Callable F,
          FuncArg... Args>
constexpr auto func(FuncName, Description, Help, F &&f, Args &&...) {
  return Function<FuncName, Description, Help, std::decay_t<F>,
                  std::decay_t<Args>...>{
      FuncName{}, Description{}, Help{}, std::forward<F>(f),
      dtl::deduce_args<F>(TypeList<Args...>{})};
}

template <Name FuncName, Callable F, FuncArg... Args>
constexpr auto func(FuncName, F &&f, Args &&...) {
  static_assert(
      type_list::list_size_v<typename function_traits<F>::arguments> ==
          sizeof...(Args),
      "All arguments of F must be named");
  return Function{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<F>(dtl::deduce_args<F>(TypeList<Args...>{})),
      std::forward<F>(f), dtl::deduce_args<F>(TypeList<Args...>{})};
}

template <Callable F, FuncArg... Args>
  requires(not std::is_pointer_v<std::decay_t<F>>)
constexpr auto func(F &&f, Args &&...) {
  static_assert(
      type_list::list_size_v<typename function_traits<F>::arguments> ==
          sizeof...(Args),
      "All arguments of F must be named");
  return Function{
      to_lower(ctti::name<std::remove_cvref_t<F>>()), NoDescription{},
      dtl::pretty_signature_name<F>(dtl::deduce_args<F>(TypeList<Args...>{})),
      std::forward<F>(f), dtl::deduce_args<F>(TypeList<Args...>{})};
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          FuncArg... Args>
constexpr auto func(FuncName, T &t, Ret (T::*mem_fun)(MemFunArgs...),
                    Args &&...) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  return Function{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<decltype(mem_fun)>(
          dtl::deduce_args<MemFunBinder<T, Ret (T::*)(MemFunArgs...)>>(
              TypeList<Args...>{})),
      MemFunBinder<T, Ret (T::*)(MemFunArgs...)>{t, mem_fun},
      dtl::deduce_args<MemFunBinder<T, Ret (T::*)(MemFunArgs...)>>(
          TypeList<Args...>{})};
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          FuncArg... Args>
constexpr auto func(FuncName, T &t, Ret (T::*mem_fun)(MemFunArgs...) noexcept,
                    Args &&...) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  return Function{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<decltype(mem_fun)>(
          dtl::deduce_args<MemFunBinder<T, Ret (T::*)(MemFunArgs...) noexcept>>(
              TypeList<Args...>{})),
      MemFunBinder<T, Ret (T::*)(MemFunArgs...) noexcept>{t, mem_fun},
      dtl::deduce_args<MemFunBinder<T, Ret (T::*)(MemFunArgs...) noexcept>>(
          TypeList<Args...>{})};
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          FuncArg... Args>
constexpr auto func(FuncName, const T &t,
                    Ret (T::*mem_fun)(MemFunArgs...) const, Args &&...) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  return Function{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<decltype(mem_fun)>(
          dtl::deduce_args<
              MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const>>(
              TypeList<Args...>{})),
      MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const>{t, mem_fun},
      dtl::deduce_args<MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const>>(
          TypeList<Args...>{})};
}

template <Name FuncName, class T, typename Ret, class... MemFunArgs,
          FuncArg... Args>
constexpr auto func(FuncName, const T &t,
                    Ret (T::*mem_fun)(MemFunArgs...) const noexcept,
                    Args &&...) {
  static_assert(sizeof...(MemFunArgs) == sizeof...(Args),
                "All arguments of mem_fun must be named");
  return Function{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<decltype(mem_fun)>(
          dtl::deduce_args<
              MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const noexcept>>(
              TypeList<Args...>{})),

      MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const noexcept>{t,
                                                                      mem_fun},
      dtl::deduce_args<
          MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const noexcept>>(
          TypeList<Args...>{})};
}

template <Name FuncName, Name Description, Name Help, class Function,
          FuncArg... Args>
struct MemberFunction {
  using arguments = TypeList<Args...>;
  static_assert(std::is_member_function_pointer_v<Function>,
                "A MemberFunctions Function template argument must be a "
                "pointer to member function");
  Function f;

  constexpr MemberFunction(FuncName, Description, Help, Function mem_fun_ptr,
                           Args...) noexcept
      : f(mem_fun_ptr) {}
  constexpr MemberFunction(FuncName, Description, Help, Function mem_fun_ptr,
                           TypeList<Args...>) noexcept
      : f(mem_fun_ptr) {}
};

template <Name FuncName, Name Description, Name Help, class Function,
          FuncArg... Args>
MemberFunction(FuncName &&, Description &&, Help &&, Function &&, Args &&...)
    -> MemberFunction<std::remove_cvref_t<FuncName>,
                      std::remove_cvref_t<Description>,
                      std::remove_cvref_t<Help>, std::remove_cvref_t<Function>,
                      std::remove_cvref_t<Args>...>;
template <Name FuncName, Name Description, Name Help, class Function,
          FuncArg... Args>
MemberFunction(FuncName &&, Description &&, Help &&, Function &&,
               TypeList<Args...>)
    -> MemberFunction<std::remove_cvref_t<FuncName>,
                      std::remove_cvref_t<Description>,
                      std::remove_cvref_t<Help>, std::remove_cvref_t<Function>,
                      std::remove_cvref_t<Args>...>;

template <Name FuncName, class MemberFunctionPointer, FuncArg... Args>
  requires std::is_member_function_pointer_v<MemberFunctionPointer>
constexpr auto mem_fun(FuncName, MemberFunctionPointer mem_fun,
                       Args &&...args) {
  constexpr auto deduced_args = dtl::deduce_args<decltype(mem_fun)>(
      TypeList<std::remove_cvref_t<Args>...>{});
  return MemberFunction{
      FuncName{}, NoDescription{},
      dtl::pretty_signature_name<decltype(mem_fun)>(deduced_args), mem_fun,
      deduced_args};
}

template <auto MemberFunctionPointer, FuncArg... Args>
  requires std::is_member_function_pointer_v<decltype(MemberFunctionPointer)>
constexpr auto mem_fun(Args &&...args) {
  constexpr auto deduced_args =
      dtl::deduce_args<decltype(MemberFunctionPointer)>(
          TypeList<std::remove_cvref_t<Args>...>{});
  return MemberFunction{
      ctti::value_name<MemberFunctionPointer>(), NoDescription{},
      dtl::pretty_signature_name<decltype(MemberFunctionPointer)>(deduced_args),
      MemberFunctionPointer, deduced_args};
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
to_cmd(T &obj,
       MemberFunction<CmdName, Description, Help, F, Args...> member_function) {
  return Function<CmdName, Description, Help, MemFunBinder<T, F>, Args...>(
      CmdName{}, Description{}, Help{}, MemFunBinder(obj, member_function.f),
      Args{}...);
}
/**
 * @brief create a Function from an object referenece and a MemberFunction
 */
template <class T, Name CmdName, Name Description, Name Help, class F,
          class... Args>
constexpr auto
to_cmd(const T &obj,
       MemberFunction<CmdName, Description, Help, F, Args...> member_function) {
  return Function<CmdName, Description, Help, MemFunBinder<const T, F>,
                  Args...>(CmdName{}, Description{}, Help{},
                           MemFunBinder(obj, member_function.f), Args{}...);
}
} // namespace dtl
} // namespace cli::funcs
#endif
