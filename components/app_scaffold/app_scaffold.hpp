#pragma once

#include <exception>
#include <tuple>
#include <utility>

namespace motion_control_lab
{

// Owns only infrastructure lifecycle ordering. Resource types are supplied by
// the concrete app and must provide start(), requestStop(), close(), and join().
// There is deliberately no app base class or erased callback/main-loop API.
template<typename... Resources>
class RuntimeLifecycle
{
public:
  explicit RuntimeLifecycle(Resources &... resources) : resources_(resources...) {}

  ~RuntimeLifecycle()
  {
    if (!finished_) stopAndJoin();
  }

  RuntimeLifecycle(const RuntimeLifecycle &) = delete;
  RuntimeLifecycle & operator=(const RuntimeLifecycle &) = delete;

  void start()
  {
    try {
      std::apply([](auto &... resource) { (resource.start(), ...); }, resources_);
      started_ = true;
    } catch (...) {
      rememberCurrentError();
      stopAndJoin();
      std::rethrow_exception(first_error_);
    }
  }

  void finish()
  {
    stopAndJoin();
    finished_ = true;
    if (first_error_) std::rethrow_exception(first_error_);
  }

private:
  template<std::size_t Index, typename Operation>
  void reverseForEach(Operation && operation) noexcept
  {
    if constexpr (Index > 0) {
      try {
        operation(std::get<Index - 1>(resources_));
      } catch (...) {
        rememberCurrentError();
      }
      reverseForEach<Index - 1>(std::forward<Operation>(operation));
    }
  }

  void stopAndJoin() noexcept
  {
    if (!started_ && !first_error_) {
      finished_ = true;
      return;
    }
    reverseForEach<sizeof...(Resources)>([](auto & resource) { resource.requestStop(); });
    reverseForEach<sizeof...(Resources)>([](auto & resource) { resource.close(); });
    reverseForEach<sizeof...(Resources)>([](auto & resource) { resource.join(); });
    finished_ = true;
  }

  void rememberCurrentError() noexcept
  {
    if (!first_error_) first_error_ = std::current_exception();
  }

  std::tuple<Resources &...> resources_;
  std::exception_ptr first_error_;
  bool started_{false};
  bool finished_{false};
};

template<typename... Resources>
RuntimeLifecycle<Resources...> makeRuntimeLifecycle(Resources &... resources)
{
  return RuntimeLifecycle<Resources...>(resources...);
}

template<typename Input, typename Scheduler, typename Terminal, typename Tui,
  typename VizSink, typename ArtifactWriter>
struct RuntimeServices
{
  Input & input;
  Scheduler & scheduler;
  Terminal & terminal;
  Tui & tui;
  VizSink & viz_sink;
  ArtifactWriter & artifact_writer;
};

template<typename Input, typename Scheduler, typename Terminal, typename Tui,
  typename VizSink, typename ArtifactWriter>
RuntimeServices<Input, Scheduler, Terminal, Tui, VizSink, ArtifactWriter> makeRuntimeServices(
  Input & input, Scheduler & scheduler, Terminal & terminal, Tui & tui,
  VizSink & viz_sink, ArtifactWriter & artifact_writer)
{
  return {input, scheduler, terminal, tui, viz_sink, artifact_writer};
}

}  // namespace motion_control_lab
