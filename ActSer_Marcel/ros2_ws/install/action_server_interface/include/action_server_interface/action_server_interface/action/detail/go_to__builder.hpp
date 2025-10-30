// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_server_interface:action/GoTo.idl
// generated code does not contain a copyright notice

#ifndef ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__BUILDER_HPP_
#define ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_server_interface/action/detail/go_to__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_Goal_pose
{
public:
  Init_GoTo_Goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_server_interface::action::GoTo_Goal pose(::action_server_interface::action::GoTo_Goal::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_Goal>()
{
  return action_server_interface::action::builder::Init_GoTo_Goal_pose();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_Result_reached
{
public:
  Init_GoTo_Result_reached()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_server_interface::action::GoTo_Result reached(::action_server_interface::action::GoTo_Result::_reached_type arg)
  {
    msg_.reached = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_Result>()
{
  return action_server_interface::action::builder::Init_GoTo_Result_reached();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_Feedback_dist_to_goal
{
public:
  Init_GoTo_Feedback_dist_to_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_server_interface::action::GoTo_Feedback dist_to_goal(::action_server_interface::action::GoTo_Feedback::_dist_to_goal_type arg)
  {
    msg_.dist_to_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_Feedback>()
{
  return action_server_interface::action::builder::Init_GoTo_Feedback_dist_to_goal();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_SendGoal_Request_goal
{
public:
  explicit Init_GoTo_SendGoal_Request_goal(::action_server_interface::action::GoTo_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_server_interface::action::GoTo_SendGoal_Request goal(::action_server_interface::action::GoTo_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_SendGoal_Request msg_;
};

class Init_GoTo_SendGoal_Request_goal_id
{
public:
  Init_GoTo_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoTo_SendGoal_Request_goal goal_id(::action_server_interface::action::GoTo_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoTo_SendGoal_Request_goal(msg_);
  }

private:
  ::action_server_interface::action::GoTo_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_SendGoal_Request>()
{
  return action_server_interface::action::builder::Init_GoTo_SendGoal_Request_goal_id();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_SendGoal_Response_stamp
{
public:
  explicit Init_GoTo_SendGoal_Response_stamp(::action_server_interface::action::GoTo_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_server_interface::action::GoTo_SendGoal_Response stamp(::action_server_interface::action::GoTo_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_SendGoal_Response msg_;
};

class Init_GoTo_SendGoal_Response_accepted
{
public:
  Init_GoTo_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoTo_SendGoal_Response_stamp accepted(::action_server_interface::action::GoTo_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GoTo_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_server_interface::action::GoTo_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_SendGoal_Response>()
{
  return action_server_interface::action::builder::Init_GoTo_SendGoal_Response_accepted();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_GetResult_Request_goal_id
{
public:
  Init_GoTo_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_server_interface::action::GoTo_GetResult_Request goal_id(::action_server_interface::action::GoTo_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_GetResult_Request>()
{
  return action_server_interface::action::builder::Init_GoTo_GetResult_Request_goal_id();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_GetResult_Response_result
{
public:
  explicit Init_GoTo_GetResult_Response_result(::action_server_interface::action::GoTo_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_server_interface::action::GoTo_GetResult_Response result(::action_server_interface::action::GoTo_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_GetResult_Response msg_;
};

class Init_GoTo_GetResult_Response_status
{
public:
  Init_GoTo_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoTo_GetResult_Response_result status(::action_server_interface::action::GoTo_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GoTo_GetResult_Response_result(msg_);
  }

private:
  ::action_server_interface::action::GoTo_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_GetResult_Response>()
{
  return action_server_interface::action::builder::Init_GoTo_GetResult_Response_status();
}

}  // namespace action_server_interface


namespace action_server_interface
{

namespace action
{

namespace builder
{

class Init_GoTo_FeedbackMessage_feedback
{
public:
  explicit Init_GoTo_FeedbackMessage_feedback(::action_server_interface::action::GoTo_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_server_interface::action::GoTo_FeedbackMessage feedback(::action_server_interface::action::GoTo_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_server_interface::action::GoTo_FeedbackMessage msg_;
};

class Init_GoTo_FeedbackMessage_goal_id
{
public:
  Init_GoTo_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoTo_FeedbackMessage_feedback goal_id(::action_server_interface::action::GoTo_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoTo_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_server_interface::action::GoTo_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_server_interface::action::GoTo_FeedbackMessage>()
{
  return action_server_interface::action::builder::Init_GoTo_FeedbackMessage_goal_id();
}

}  // namespace action_server_interface

#endif  // ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__BUILDER_HPP_
