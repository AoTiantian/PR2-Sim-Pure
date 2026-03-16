#ifndef UNITREE_SDK2_BRIDGE_H
#define UNITREE_SDK2_BRIDGE_H

#include <array>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/pr2/pr2_cmd_.hpp>
#include <unitree/idl/pr2/pr2_state_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);
    ~UnitreeSdk2Bridge() = default;

    void PR2CmdHandler(const void *msg);
    void PublishPR2State();

    void Run();
    void PrintSceneInformation();
    void SetupJoystick(std::string device, std::string js_type, int bits);

private:
    void CacheActuatorIds();
    double GetSensorData(const std::string &name, int offset = 0) const;

    mjData *mj_data_;
    mjModel *mj_model_;

    bool actuator_ids_cached_ = false;
    bool pr2_loaded_ = false;

    int torso_id_ = -1;
    int r_gripper_id_ = -1;
    int l_gripper_id_ = -1;
    int head_pan_id_ = -1;
    int head_tilt_id_ = -1;
    int laser_tilt_id_ = -1;

    std::array<int, 7> r_arm_ids_{};
    std::array<int, 7> l_arm_ids_{};
    std::array<int, 4> caster_ids_{};
    std::array<int, 8> wheel_ids_{};

    pr2cmd::msg::dds_::PR2SensorState pr2_state_{};

    unitree::robot::ChannelSubscriberPtr<pr2cmd::msg::dds_::PR2ActuatorCmd> pr2_cmd_suber_;
    unitree::robot::ChannelPublisherPtr<pr2cmd::msg::dds_::PR2SensorState> pr2_state_puber_;
    unitree::common::ThreadPtr pr2_state_thread_;
};

#endif
