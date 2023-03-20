#pragma once

#include "teleoperate_robot/keyboard_reader.hpp"

#include "rclcpp/rclcpp.hpp"



/* ========================================================================== */
/*                                 DECLARATION                                */
/* ========================================================================== */

namespace teleoperate_robot {

template <class T> class TeleopRobot : public rclcpp::Node {
public:
    TeleopRobot();

    /// Function that handles the keyboard input and publishes the command.
    void teleop_callback();

    void shutdown() {keyboard_reader_.shutdown();};
protected:
    virtual void print_instructions() = 0;

    void read_key(char *c)
    {
        try {
            keyboard_reader_.read_one(c);
        } catch (const std::runtime_error &) {
            perror("read():");
        }
    };

    virtual bool process_key(const char c) = 0;

    virtual void update_message() = 0;

    virtual bool initialize_message() {return true;};

    bool init_complete = false;

    KeyboardReader keyboard_reader_;

    T msg_;

    typename rclcpp::Publisher<T>::SharedPtr pub_;

    const rclcpp::Duration timer_period_ = rclcpp::Duration::from_seconds(1./25.);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};



/* ========================================================================== */
/*                                 DEFINITION                                 */
/* ========================================================================== */

/* =============================== Constructor ============================== */

template <class T> TeleopRobot<T>::TeleopRobot()
: Node("teleop_robot")
{
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period_, std::bind(&TeleopRobot<T>::teleop_callback, this), timer_cb_group_);
}


/* ============================= Teleop_callback ============================ */

template <class T> void TeleopRobot<T>::teleop_callback()
{
    char c;
    bool dirty = false;

    read_key(&c);

    dirty = process_key(c);

    if (dirty == true) {
        if (init_complete == false) {
            init_complete = initialize_message();
        }

        if (init_complete == true) {
            print_instructions();

            update_message();

            pub_->publish(msg_);
        }
    }
}

} // namespace teleop_robot