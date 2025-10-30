// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_server_interface:action/GoTo.idl
// generated code does not contain a copyright notice

#ifndef ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__STRUCT_H_
#define ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_Goal
{
  geometry_msgs__msg__Pose2D pose;
} action_server_interface__action__GoTo_Goal;

// Struct for a sequence of action_server_interface__action__GoTo_Goal.
typedef struct action_server_interface__action__GoTo_Goal__Sequence
{
  action_server_interface__action__GoTo_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_Result
{
  bool reached;
} action_server_interface__action__GoTo_Result;

// Struct for a sequence of action_server_interface__action__GoTo_Result.
typedef struct action_server_interface__action__GoTo_Result__Sequence
{
  action_server_interface__action__GoTo_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_Feedback
{
  float dist_to_goal;
} action_server_interface__action__GoTo_Feedback;

// Struct for a sequence of action_server_interface__action__GoTo_Feedback.
typedef struct action_server_interface__action__GoTo_Feedback__Sequence
{
  action_server_interface__action__GoTo_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_server_interface/action/detail/go_to__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_server_interface__action__GoTo_Goal goal;
} action_server_interface__action__GoTo_SendGoal_Request;

// Struct for a sequence of action_server_interface__action__GoTo_SendGoal_Request.
typedef struct action_server_interface__action__GoTo_SendGoal_Request__Sequence
{
  action_server_interface__action__GoTo_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_server_interface__action__GoTo_SendGoal_Response;

// Struct for a sequence of action_server_interface__action__GoTo_SendGoal_Response.
typedef struct action_server_interface__action__GoTo_SendGoal_Response__Sequence
{
  action_server_interface__action__GoTo_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_server_interface__action__GoTo_GetResult_Request;

// Struct for a sequence of action_server_interface__action__GoTo_GetResult_Request.
typedef struct action_server_interface__action__GoTo_GetResult_Request__Sequence
{
  action_server_interface__action__GoTo_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_server_interface/action/detail/go_to__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_GetResult_Response
{
  int8_t status;
  action_server_interface__action__GoTo_Result result;
} action_server_interface__action__GoTo_GetResult_Response;

// Struct for a sequence of action_server_interface__action__GoTo_GetResult_Response.
typedef struct action_server_interface__action__GoTo_GetResult_Response__Sequence
{
  action_server_interface__action__GoTo_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_server_interface/action/detail/go_to__struct.h"

/// Struct defined in action/GoTo in the package action_server_interface.
typedef struct action_server_interface__action__GoTo_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_server_interface__action__GoTo_Feedback feedback;
} action_server_interface__action__GoTo_FeedbackMessage;

// Struct for a sequence of action_server_interface__action__GoTo_FeedbackMessage.
typedef struct action_server_interface__action__GoTo_FeedbackMessage__Sequence
{
  action_server_interface__action__GoTo_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_server_interface__action__GoTo_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_SERVER_INTERFACE__ACTION__DETAIL__GO_TO__STRUCT_H_
