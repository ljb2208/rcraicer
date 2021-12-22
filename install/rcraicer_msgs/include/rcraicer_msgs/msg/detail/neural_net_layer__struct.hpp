// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__NeuralNetLayer __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__NeuralNetLayer __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NeuralNetLayer_
{
  using Type = NeuralNetLayer_<ContainerAllocator>;

  explicit NeuralNetLayer_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit NeuralNetLayer_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _name_type name;
  using _weight_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _weight_type weight;
  using _bias_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _bias_type bias;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__weight(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->weight = _arg;
    return *this;
  }
  Type & set__bias(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->bias = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__NeuralNetLayer
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__NeuralNetLayer
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NeuralNetLayer_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->weight != other.weight) {
      return false;
    }
    if (this->bias != other.bias) {
      return false;
    }
    return true;
  }
  bool operator!=(const NeuralNetLayer_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NeuralNetLayer_

// alias to use template instance with default allocator
using NeuralNetLayer =
  rcraicer_msgs::msg::NeuralNetLayer_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_HPP_
