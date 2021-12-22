// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'network'
#include "rcraicer_msgs/msg/detail/neural_net_layer__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__NeuralNetModel __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__NeuralNetModel __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NeuralNetModel_
{
  using Type = NeuralNetModel_<ContainerAllocator>;

  explicit NeuralNetModel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_layers = 0l;
    }
  }

  explicit NeuralNetModel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_layers = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _num_layers_type =
    int32_t;
  _num_layers_type num_layers;
  using _network_type =
    std::vector<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>, typename ContainerAllocator::template rebind<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>::other>;
  _network_type network;
  using _structure_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _structure_type structure;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__num_layers(
    const int32_t & _arg)
  {
    this->num_layers = _arg;
    return *this;
  }
  Type & set__network(
    const std::vector<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>, typename ContainerAllocator::template rebind<rcraicer_msgs::msg::NeuralNetLayer_<ContainerAllocator>>::other> & _arg)
  {
    this->network = _arg;
    return *this;
  }
  Type & set__structure(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->structure = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__NeuralNetModel
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__NeuralNetModel
    std::shared_ptr<rcraicer_msgs::msg::NeuralNetModel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NeuralNetModel_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->num_layers != other.num_layers) {
      return false;
    }
    if (this->network != other.network) {
      return false;
    }
    if (this->structure != other.structure) {
      return false;
    }
    return true;
  }
  bool operator!=(const NeuralNetModel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NeuralNetModel_

// alias to use template instance with default allocator
using NeuralNetModel =
  rcraicer_msgs::msg::NeuralNetModel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_HPP_
