// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from action_server_interface:action/GoTo.idl
// generated code does not contain a copyright notice

#ifndef ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__TRAITS_HPP_
#define ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "action_server_interface/action/detail/go_to__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_Goal & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_Goal>()
{
  return "action_server_interface::action::GoTo_Goal";
}

template<>
inline const char * name<action_server_interface::action::GoTo_Goal>()
{
  return "action_server_interface/action/GoTo_Goal";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_Goal>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_Goal>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: reached
  {
    out << "reached: ";
    rosidl_generator_traits::value_to_yaml(msg.reached, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reached
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reached: ";
    rosidl_generator_traits::value_to_yaml(msg.reached, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_Result & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_Result>()
{
  return "action_server_interface::action::GoTo_Result";
}

template<>
inline const char * name<action_server_interface::action::GoTo_Result>()
{
  return "action_server_interface/action/GoTo_Result";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_server_interface::action::GoTo_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: dist_to_goal
  {
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dist_to_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_Feedback & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_Feedback>()
{
  return "action_server_interface::action::GoTo_Feedback";
}

template<>
inline const char * name<action_server_interface::action::GoTo_Feedback>()
{
  return "action_server_interface/action/GoTo_Feedback";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_server_interface::action::GoTo_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "action_server_interface/action/detail/go_to__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_SendGoal_Request & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_SendGoal_Request>()
{
  return "action_server_interface::action::GoTo_SendGoal_Request";
}

template<>
inline const char * name<action_server_interface::action::GoTo_SendGoal_Request>()
{
  return "action_server_interface/action/GoTo_SendGoal_Request";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<action_server_interface::action::GoTo_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<action_server_interface::action::GoTo_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_SendGoal_Response & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_SendGoal_Response>()
{
  return "action_server_interface::action::GoTo_SendGoal_Response";
}

template<>
inline const char * name<action_server_interface::action::GoTo_SendGoal_Response>()
{
  return "action_server_interface/action/GoTo_SendGoal_Response";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_server_interface::action::GoTo_SendGoal>()
{
  return "action_server_interface::action::GoTo_SendGoal";
}

template<>
inline const char * name<action_server_interface::action::GoTo_SendGoal>()
{
  return "action_server_interface/action/GoTo_SendGoal";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<action_server_interface::action::GoTo_SendGoal_Request>::value &&
    has_fixed_size<action_server_interface::action::GoTo_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<action_server_interface::action::GoTo_SendGoal_Request>::value &&
    has_bounded_size<action_server_interface::action::GoTo_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<action_server_interface::action::GoTo_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<action_server_interface::action::GoTo_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_server_interface::action::GoTo_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_GetResult_Request & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_GetResult_Request>()
{
  return "action_server_interface::action::GoTo_GetResult_Request";
}

template<>
inline const char * name<action_server_interface::action::GoTo_GetResult_Request>()
{
  return "action_server_interface/action/GoTo_GetResult_Request";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "action_server_interface/action/detail/go_to__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_GetResult_Response & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_GetResult_Response>()
{
  return "action_server_interface::action::GoTo_GetResult_Response";
}

template<>
inline const char * name<action_server_interface::action::GoTo_GetResult_Response>()
{
  return "action_server_interface/action/GoTo_GetResult_Response";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<action_server_interface::action::GoTo_Result>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<action_server_interface::action::GoTo_Result>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_server_interface::action::GoTo_GetResult>()
{
  return "action_server_interface::action::GoTo_GetResult";
}

template<>
inline const char * name<action_server_interface::action::GoTo_GetResult>()
{
  return "action_server_interface/action/GoTo_GetResult";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<action_server_interface::action::GoTo_GetResult_Request>::value &&
    has_fixed_size<action_server_interface::action::GoTo_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<action_server_interface::action::GoTo_GetResult_Request>::value &&
    has_bounded_size<action_server_interface::action::GoTo_GetResult_Response>::value
  >
{
};

template<>
struct is_service<action_server_interface::action::GoTo_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<action_server_interface::action::GoTo_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_server_interface::action::GoTo_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "action_server_interface/action/detail/go_to__traits.hpp"

namespace action_server_interface
{

namespace action
{

inline void to_flow_style_yaml(
  const GoTo_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoTo_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoTo_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_server_interface

namespace rosidl_generator_traits
{

[[deprecated("use action_server_interface::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_server_interface::action::GoTo_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_server_interface::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_server_interface::action::to_yaml() instead")]]
inline std::string to_yaml(const action_server_interface::action::GoTo_FeedbackMessage & msg)
{
  return action_server_interface::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_server_interface::action::GoTo_FeedbackMessage>()
{
  return "action_server_interface::action::GoTo_FeedbackMessage";
}

template<>
inline const char * name<action_server_interface::action::GoTo_FeedbackMessage>()
{
  return "action_server_interface/action/GoTo_FeedbackMessage";
}

template<>
struct has_fixed_size<action_server_interface::action::GoTo_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<action_server_interface::action::GoTo_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_server_interface::action::GoTo_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<action_server_interface::action::GoTo_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_server_interface::action::GoTo_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<action_server_interface::action::GoTo>
  : std::true_type
{
};

template<>
struct is_action_goal<action_server_interface::action::GoTo_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<action_server_interface::action::GoTo_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<action_server_interface::action::GoTo_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__TRAITS_HPP_
