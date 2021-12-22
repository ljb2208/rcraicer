// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__FUNCTIONS_H_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rcraicer_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rcraicer_msgs/msg/detail/neural_net_model__struct.h"

/// Initialize msg/NeuralNetModel message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcraicer_msgs__msg__NeuralNetModel
 * )) before or use
 * rcraicer_msgs__msg__NeuralNetModel__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
bool
rcraicer_msgs__msg__NeuralNetModel__init(rcraicer_msgs__msg__NeuralNetModel * msg);

/// Finalize msg/NeuralNetModel message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
void
rcraicer_msgs__msg__NeuralNetModel__fini(rcraicer_msgs__msg__NeuralNetModel * msg);

/// Create msg/NeuralNetModel message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcraicer_msgs__msg__NeuralNetModel__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
rcraicer_msgs__msg__NeuralNetModel *
rcraicer_msgs__msg__NeuralNetModel__create();

/// Destroy msg/NeuralNetModel message.
/**
 * It calls
 * rcraicer_msgs__msg__NeuralNetModel__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
void
rcraicer_msgs__msg__NeuralNetModel__destroy(rcraicer_msgs__msg__NeuralNetModel * msg);


/// Initialize array of msg/NeuralNetModel messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcraicer_msgs__msg__NeuralNetModel__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
bool
rcraicer_msgs__msg__NeuralNetModel__Sequence__init(rcraicer_msgs__msg__NeuralNetModel__Sequence * array, size_t size);

/// Finalize array of msg/NeuralNetModel messages.
/**
 * It calls
 * rcraicer_msgs__msg__NeuralNetModel__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
void
rcraicer_msgs__msg__NeuralNetModel__Sequence__fini(rcraicer_msgs__msg__NeuralNetModel__Sequence * array);

/// Create array of msg/NeuralNetModel messages.
/**
 * It allocates the memory for the array and calls
 * rcraicer_msgs__msg__NeuralNetModel__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
rcraicer_msgs__msg__NeuralNetModel__Sequence *
rcraicer_msgs__msg__NeuralNetModel__Sequence__create(size_t size);

/// Destroy array of msg/NeuralNetModel messages.
/**
 * It calls
 * rcraicer_msgs__msg__NeuralNetModel__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcraicer_msgs
void
rcraicer_msgs__msg__NeuralNetModel__Sequence__destroy(rcraicer_msgs__msg__NeuralNetModel__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__FUNCTIONS_H_
