#include "irc_step_motion_executor/sdk_executor_core.hpp"

#include <json-c/json.h>

#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

struct ParsedRequest
{
  MotionRequest request;
  std::uint64_t timeout_ms{0};
};

MotionStatus rejected_status(
  const std::string & error_code, const std::string & message)
{
  MotionStatus status;
  status.error_code = error_code;
  status.message = message;
  return status;
}

MotionStatus status_for_request(
  const MotionRequest & request, const std::string & status_value,
  const std::string & error_code, const std::string & message)
{
  MotionStatus status;
  status.status = status_value;
  status.action = request.action;
  status.command_id = request.command_id;
  status.event_id = request.event_id;
  status.request_id = request.request_id;
  status.motion_id = request.motion_id;
  status.error_code = error_code;
  status.message = message;
  return status;
}

bool get_required_int64(
  json_object * object, const char * key, std::int64_t & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
    json_object_get_type(value) != json_type_int)
  {
    return false;
  }
  output = json_object_get_int64(value);
  return true;
}

bool get_required_positive_uint64(
  json_object * object, const char * key, std::uint64_t & output)
{
  std::int64_t parsed = 0;
  if (!get_required_int64(object, key, parsed) || parsed <= 0) {
    return false;
  }
  output = static_cast<std::uint64_t>(parsed);
  return true;
}

bool get_required_string(
  json_object * object, const char * key, std::string & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
    json_object_get_type(value) != json_type_string)
  {
    return false;
  }
  output = json_object_get_string(value);
  return !output.empty();
}

bool get_required_nullable_int(
  json_object * object, const char * key,
  std::optional<std::int64_t> & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value)) {
    return false;
  }
  if (json_object_get_type(value) == json_type_null) {
    output.reset();
    return true;
  }
  if (json_object_get_type(value) != json_type_int) {
    return false;
  }
  output = json_object_get_int64(value);
  return true;
}

std::optional<ParsedRequest> parse_request(
  const std::string & payload, std::string & error_message)
{
  json_tokener * tokener = json_tokener_new();
  json_object * object =
    json_tokener_parse_ex(tokener, payload.c_str(), payload.size());
  const json_tokener_error parse_error = json_tokener_get_error(tokener);
  json_tokener_free(tokener);

  if (parse_error != json_tokener_success || object == nullptr) {
    if (object != nullptr) {
      json_object_put(object);
    }
    error_message = "invalid JSON request";
    return std::nullopt;
  }
  if (json_object_get_type(object) != json_type_object) {
    json_object_put(object);
    error_message = "request must be a JSON object";
    return std::nullopt;
  }

  ParsedRequest parsed;
  std::string action;
  const bool valid =
    get_required_string(object, "action", action) &&
    get_required_nullable_int(
      object, "command_id", parsed.request.command_id) &&
    get_required_nullable_int(
      object, "event_id", parsed.request.event_id) &&
    get_required_int64(
      object, "request_id", parsed.request.request_id) &&
    get_required_string(
      object, "motion_id", parsed.request.motion_id) &&
    get_required_positive_uint64(
      object, "timeout_ms", parsed.timeout_ms);
  json_object_put(object);

  if (!valid) {
    error_message =
      "request fields are missing or invalid; timeout_ms must be positive";
    return std::nullopt;
  }
  parsed.request.action = std::move(action);
  return parsed;
}

std::optional<std::int64_t> parse_cancel_request_id(
  const std::string & payload, std::string & error_message)
{
  json_tokener * tokener = json_tokener_new();
  json_object * object =
    json_tokener_parse_ex(tokener, payload.c_str(), payload.size());
  const json_tokener_error parse_error = json_tokener_get_error(tokener);
  json_tokener_free(tokener);

  if (parse_error != json_tokener_success || object == nullptr ||
    json_object_get_type(object) != json_type_object)
  {
    if (object != nullptr) {
      json_object_put(object);
    }
    error_message = "invalid cancel JSON";
    return std::nullopt;
  }

  std::int64_t request_id = 0;
  const bool valid = get_required_int64(object, "request_id", request_id);
  json_object_put(object);
  if (!valid) {
    error_message = "cancel request_id must be an integer";
    return std::nullopt;
  }
  return request_id;
}

std::string exception_message(
  const std::string & operation, const std::exception & exception)
{
  return operation + " exception: " + exception.what();
}

}  // namespace

SdkExecutorCore::SdkExecutorCore(
  MotionAliasCatalog catalog, MotionBackend & backend)
: catalog_(std::move(catalog)), backend_(backend)
{
}

MotionStatus SdkExecutorCore::handle_request(
  const std::string & payload, std::uint64_t now_ms)
{
  std::string error_message;
  const auto parsed = parse_request(payload, error_message);
  if (!parsed) {
    return rejected_status("INVALID_REQUEST", error_message);
  }

  if (active_) {
    return status_for_request(
      parsed->request, "REJECTED", "BUSY",
      "another motion request is already active");
  }

  const auto resolved_motion = catalog_.resolve(parsed->request.motion_id);
  if (!resolved_motion) {
    return status_for_request(
      parsed->request, "REJECTED", "INVALID_MOTION",
      "unsupported motion_id; backend was not called");
  }

  try {
    const BackendStartResult start_result =
      backend_.start_motion(*resolved_motion);
    if (!start_result.accepted) {
      const std::string error_code = start_result.error_code.empty() ?
        "START_REJECTED" : start_result.error_code;
      return status_for_request(
        parsed->request, "REJECTED", error_code, start_result.message);
    }
  } catch (const std::exception & exception) {
    return status_for_request(
      parsed->request, "FAILED", "BACKEND_EXCEPTION",
      exception_message("start_motion", exception));
  } catch (...) {
    return status_for_request(
      parsed->request, "FAILED", "BACKEND_EXCEPTION",
      "start_motion threw an unknown exception");
  }

  active_ = ActiveRequest{
    parsed->request, now_ms, parsed->timeout_ms};
  return status_for_active("RUNNING", "", "motion start accepted");
}

MotionStatus SdkExecutorCore::handle_cancel(const std::string & payload)
{
  std::string error_message;
  const auto request_id = parse_cancel_request_id(payload, error_message);
  if (!request_id) {
    return rejected_status("INVALID_REQUEST", error_message);
  }
  if (!active_) {
    MotionStatus status =
      rejected_status("NOT_RUNNING", "no active motion request");
    status.request_id = *request_id;
    return status;
  }
  if (*request_id != active_->request.request_id) {
    return status_for_active(
      "REJECTED", "STALE_REQUEST",
      "cancel request_id does not match the active request");
  }

  try {
    const BackendCancelResult cancel_result = backend_.cancel_motion();
    if (!cancel_result.accepted) {
      const std::string error_code = cancel_result.error_code.empty() ?
        "CANCEL_REJECTED" : cancel_result.error_code;
      return status_for_active(
        "REJECTED", error_code, cancel_result.message);
    }
  } catch (const std::exception & exception) {
    MotionStatus status = status_for_active(
      "FAILED", "BACKEND_EXCEPTION",
      exception_message("cancel_motion", exception));
    clear_active();
    return status;
  } catch (...) {
    MotionStatus status = status_for_active(
      "FAILED", "BACKEND_EXCEPTION",
      "cancel_motion threw an unknown exception");
    clear_active();
    return status;
  }

  return status_for_active("RUNNING", "", "cancel requested");
}

std::optional<MotionStatus> SdkExecutorCore::poll(std::uint64_t now_ms)
{
  if (!active_) {
    return std::nullopt;
  }

  const bool timeout_reached =
    now_ms >= active_->started_at_ms &&
    now_ms - active_->started_at_ms >= active_->timeout_ms;
  if (timeout_reached) {
    std::string message = "motion timeout; backend cancel requested";
    try {
      const BackendCancelResult cancel_result = backend_.cancel_motion();
      if (!cancel_result.message.empty()) {
        message += ": " + cancel_result.message;
      }
    } catch (const std::exception & exception) {
      message += "; " + exception_message("cancel_motion", exception);
    } catch (...) {
      message += "; cancel_motion threw an unknown exception";
    }
    MotionStatus status = status_for_active(
      "FAILED", "TIMEOUT", message);
    clear_active();
    return status;
  }

  BackendStatus backend_status;
  try {
    backend_status = backend_.poll_status();
  } catch (const std::exception & exception) {
    MotionStatus status = status_for_active(
      "FAILED", "BACKEND_EXCEPTION",
      exception_message("poll_status", exception));
    clear_active();
    return status;
  } catch (...) {
    MotionStatus status = status_for_active(
      "FAILED", "BACKEND_EXCEPTION",
      "poll_status threw an unknown exception");
    clear_active();
    return status;
  }

  switch (backend_status.state) {
    case BackendState::RUNNING:
      return status_for_active(
        "RUNNING", backend_status.error_code, backend_status.message);
    case BackendState::SETTLING:
      return status_for_active(
        "RUNNING", backend_status.error_code,
        backend_status.message.empty() ?
        "backend settling" : "backend settling: " + backend_status.message);
    case BackendState::SUCCEEDED:
    {
      MotionStatus status = status_for_active(
        "SUCCEEDED", backend_status.error_code, backend_status.message);
      clear_active();
      return status;
    }
    case BackendState::CANCELLED:
    {
      MotionStatus status = status_for_active(
        "CANCELLED", backend_status.error_code, backend_status.message);
      clear_active();
      return status;
    }
    case BackendState::FAILED:
    {
      MotionStatus status = status_for_active(
        "FAILED",
        backend_status.error_code.empty() ?
        "BACKEND_FAILED" : backend_status.error_code,
        backend_status.message);
      clear_active();
      return status;
    }
    case BackendState::IDLE:
    default:
    {
      MotionStatus status = status_for_active(
        "FAILED", "BACKEND_UNEXPECTED_IDLE",
        "backend became idle while a request was active");
      clear_active();
      return status;
    }
  }
}

bool SdkExecutorCore::has_active_request() const
{
  return active_.has_value();
}

MotionStatus SdkExecutorCore::status_for_active(
  const std::string & status, const std::string & error_code,
  const std::string & message) const
{
  return status_for_request(
    active_->request, status, error_code, message);
}

void SdkExecutorCore::clear_active()
{
  active_.reset();
}

}  // namespace irc_step_motion_executor
