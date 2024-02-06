#include "rclcpp/rclcpp.hpp"

namespace dlo {

template <typename T>
struct identity {
    typedef T type;
};

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param,
                   const typename identity<T>::type& default_value) {
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
}

}  // namespace dlio
