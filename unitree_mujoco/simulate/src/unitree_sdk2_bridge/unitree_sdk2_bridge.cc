#include "unitree_sdk2_bridge.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data)
    : mj_data_(data), mj_model_(model)
{
    r_arm_ids_.fill(-1);
    l_arm_ids_.fill(-1);
    caster_ids_.fill(-1);
    wheel_ids_.fill(-1);

    CacheActuatorIds();

    pr2_cmd_suber_.reset(new unitree::robot::ChannelSubscriber<pr2cmd::msg::dds_::PR2ActuatorCmd>("pr2/cmd"));
    pr2_cmd_suber_->InitChannel(std::bind(&UnitreeSdk2Bridge::PR2CmdHandler, this, std::placeholders::_1), 1);

    pr2_state_puber_.reset(new unitree::robot::ChannelPublisher<pr2cmd::msg::dds_::PR2SensorState>("pr2/state"));
    pr2_state_puber_->InitChannel();

    pr2_state_thread_ = unitree::common::CreateRecurrentThreadEx(
        "pr2_state", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishPR2State, this);
}

void UnitreeSdk2Bridge::CacheActuatorIds()
{
    if (actuator_ids_cached_ || mj_model_ == nullptr)
    {
        return;
    }

    actuator_ids_cached_ = true;

    auto get_id = [this](const std::string &name) {
        return mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
    };

    torso_id_ = get_id("torso_lift_tau");
    pr2_loaded_ = torso_id_ != -1;

    if (!pr2_loaded_)
    {
        std::cerr << "[UnitreeBridge] PR2 actuators not found in scene." << std::endl;
        return;
    }

    r_gripper_id_ = get_id("r_gripper_pos");
    l_gripper_id_ = get_id("l_gripper_pos");
    head_pan_id_ = get_id("head_pan_pos");
    head_tilt_id_ = get_id("head_tilt_pos");
    laser_tilt_id_ = get_id("laser_tilt_pos");

    const std::array<std::string, 7> arm_suffixes = {
        "shoulder_pan_tau", "shoulder_lift_tau", "upper_arm_roll_tau", "elbow_flex_tau",
        "forearm_roll_tau", "wrist_flex_tau", "wrist_roll_tau"};

    for (size_t i = 0; i < arm_suffixes.size(); ++i)
    {
        r_arm_ids_[i] = get_id("r_" + arm_suffixes[i]);
        l_arm_ids_[i] = get_id("l_" + arm_suffixes[i]);
    }

    caster_ids_[0] = get_id("fl_caster_steer");
    caster_ids_[1] = get_id("fr_caster_steer");
    caster_ids_[2] = get_id("bl_caster_steer");
    caster_ids_[3] = get_id("br_caster_steer");

    wheel_ids_[0] = get_id("fl_caster_l_wheel_vel");
    wheel_ids_[1] = get_id("fl_caster_r_wheel_vel");
    wheel_ids_[2] = get_id("fr_caster_l_wheel_vel");
    wheel_ids_[3] = get_id("fr_caster_r_wheel_vel");
    wheel_ids_[4] = get_id("bl_caster_l_wheel_vel");
    wheel_ids_[5] = get_id("bl_caster_r_wheel_vel");
    wheel_ids_[6] = get_id("br_caster_l_wheel_vel");
    wheel_ids_[7] = get_id("br_caster_r_wheel_vel");

    std::cout << "[UnitreeBridge] PR2 interfaces initialized." << std::endl;
}

void UnitreeSdk2Bridge::PR2CmdHandler(const void *msg)
{
    if (mj_data_ == nullptr)
    {
        return;
    }

    CacheActuatorIds();
    if (!pr2_loaded_)
    {
        return;
    }

    const auto *cmd = static_cast<const pr2cmd::msg::dds_::PR2ActuatorCmd *>(msg);

    if (r_gripper_id_ != -1)
        mj_data_->ctrl[r_gripper_id_] = cmd->r_gripper_pos();
    if (l_gripper_id_ != -1)
        mj_data_->ctrl[l_gripper_id_] = cmd->l_gripper_pos();
    if (head_pan_id_ != -1)
        mj_data_->ctrl[head_pan_id_] = cmd->head_pan_pos();
    if (head_tilt_id_ != -1)
        mj_data_->ctrl[head_tilt_id_] = cmd->head_tilt_pos();
    if (laser_tilt_id_ != -1)
        mj_data_->ctrl[laser_tilt_id_] = cmd->laser_tilt_pos();

    if (caster_ids_[0] != -1)
        mj_data_->ctrl[caster_ids_[0]] = cmd->fl_caster_steer();
    if (caster_ids_[1] != -1)
        mj_data_->ctrl[caster_ids_[1]] = cmd->fr_caster_steer();
    if (caster_ids_[2] != -1)
        mj_data_->ctrl[caster_ids_[2]] = cmd->bl_caster_steer();
    if (caster_ids_[3] != -1)
        mj_data_->ctrl[caster_ids_[3]] = cmd->br_caster_steer();

    if (wheel_ids_[0] != -1)
        mj_data_->ctrl[wheel_ids_[0]] = cmd->fl_caster_l_wheel_vel();
    if (wheel_ids_[1] != -1)
        mj_data_->ctrl[wheel_ids_[1]] = cmd->fl_caster_r_wheel_vel();
    if (wheel_ids_[2] != -1)
        mj_data_->ctrl[wheel_ids_[2]] = cmd->fr_caster_l_wheel_vel();
    if (wheel_ids_[3] != -1)
        mj_data_->ctrl[wheel_ids_[3]] = cmd->fr_caster_r_wheel_vel();
    if (wheel_ids_[4] != -1)
        mj_data_->ctrl[wheel_ids_[4]] = cmd->bl_caster_l_wheel_vel();
    if (wheel_ids_[5] != -1)
        mj_data_->ctrl[wheel_ids_[5]] = cmd->bl_caster_r_wheel_vel();
    if (wheel_ids_[6] != -1)
        mj_data_->ctrl[wheel_ids_[6]] = cmd->br_caster_l_wheel_vel();
    if (wheel_ids_[7] != -1)
        mj_data_->ctrl[wheel_ids_[7]] = cmd->br_caster_r_wheel_vel();

    if (torso_id_ != -1)
        mj_data_->ctrl[torso_id_] = cmd->torso_lift_tau();

    for (size_t i = 0; i < r_arm_ids_.size(); ++i)
    {
        if (r_arm_ids_[i] != -1)
            mj_data_->ctrl[r_arm_ids_[i]] = cmd->r_arm_tau()[i];
        if (l_arm_ids_[i] != -1)
            mj_data_->ctrl[l_arm_ids_[i]] = cmd->l_arm_tau()[i];
    }
}

double UnitreeSdk2Bridge::GetSensorData(const std::string &name, int offset) const
{
    if (mj_model_ == nullptr || mj_data_ == nullptr)
    {
        return 0.0;
    }

    const int sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, name.c_str());
    if (sensor_id == -1)
    {
        return 0.0;
    }

    return mj_data_->sensordata[mj_model_->sensor_adr[sensor_id] + offset];
}

void UnitreeSdk2Bridge::PublishPR2State()
{
    if (mj_data_ == nullptr || mj_model_ == nullptr || !pr2_state_puber_)
    {
        return;
    }

    pr2_state_.torso_lift_q() = GetSensorData("torso_lift_q");
    pr2_state_.torso_lift_dq() = GetSensorData("torso_lift_dq");

    const std::array<std::string, 7> arm_suffixes = {
        "shoulder_pan", "shoulder_lift", "upper_arm_roll", "elbow_flex",
        "forearm_roll", "wrist_flex", "wrist_roll"};

    for (size_t i = 0; i < arm_suffixes.size(); ++i)
    {
        const std::string r_name = "r_" + arm_suffixes[i];
        const std::string l_name = "l_" + arm_suffixes[i];

        pr2_state_.r_arm_q()[i] = GetSensorData(r_name + "_q");
        pr2_state_.r_arm_dq()[i] = GetSensorData(r_name + "_dq");

        pr2_state_.l_arm_q()[i] = GetSensorData(l_name + "_q");
        pr2_state_.l_arm_dq()[i] = GetSensorData(l_name + "_dq");
    }

    const std::array<std::string, 4> caster_prefixes = {"fl_caster", "fr_caster", "bl_caster", "br_caster"};
    for (size_t i = 0; i < caster_prefixes.size(); ++i)
    {
        const std::string prefix = caster_prefixes[i];

        pr2_state_.caster_rotation_q()[i] = GetSensorData(prefix + "_rotation_q");
        pr2_state_.caster_l_wheel_q()[i] = GetSensorData(prefix + "_l_wheel_q");
        pr2_state_.caster_l_wheel_dq()[i] = GetSensorData(prefix + "_l_wheel_dq");
        pr2_state_.caster_r_wheel_q()[i] = GetSensorData(prefix + "_r_wheel_q");
        pr2_state_.caster_r_wheel_dq()[i] = GetSensorData(prefix + "_r_wheel_dq");
    }

    for (int i = 0; i < 3; ++i)
    {
        pr2_state_.pr2_base_pos()[i] = GetSensorData("pr2/base_pos", i);
        pr2_state_.desk_base_pos()[i] = GetSensorData("desk/base_pos", i);
        pr2_state_.right_force()[i] = GetSensorData("pr2/right_force", i);
        pr2_state_.right_torque()[i] = GetSensorData("pr2/right_torque", i);
        pr2_state_.left_force()[i] = GetSensorData("pr2/left_force", i);
        pr2_state_.left_torque()[i] = GetSensorData("pr2/left_torque", i);
    }

    for (int i = 0; i < 4; ++i)
    {
        pr2_state_.pr2_base_quat()[i] = GetSensorData("pr2/base_quat", i);
        pr2_state_.desk_base_quat()[i] = GetSensorData("desk/base_quat", i);
    }

    pr2_state_puber_->Write(pr2_state_);
}

void UnitreeSdk2Bridge::Run()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void UnitreeSdk2Bridge::SetupJoystick(std::string device, std::string js_type, int bits)
{
    (void)device;
    (void)js_type;
    (void)bits;
    std::cout << "[UnitreeBridge] Joystick interface is disabled in PR2-only build." << std::endl;
}

void UnitreeSdk2Bridge::PrintSceneInformation()
{
    std::cout << "\n<<------------- Link ------------->>\n";
    for (int i = 0; i < mj_model_->nbody; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_BODY, i);
        if (name)
        {
            std::cout << "link_index: " << i << ", name: " << name << '\n';
        }
    }

    std::cout << "\n<<------------- Joint ------------->>\n";
    for (int i = 0; i < mj_model_->njnt; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name)
        {
            std::cout << "joint_index: " << i << ", name: " << name << '\n';
        }
    }

    std::cout << "\n<<------------- Actuator ------------->>\n";
    for (int i = 0; i < mj_model_->nu; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
        if (name)
        {
            std::cout << "actuator_index: " << i << ", name: " << name << '\n';
        }
    }

    std::cout << "\n<<------------- Sensor ------------->>\n";
    int index = 0;
    for (int i = 0; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (name)
        {
            std::cout << "sensor_index: " << index << ", name: " << name << ", dim: " << mj_model_->sensor_dim[i] << '\n';
        }
        index += mj_model_->sensor_dim[i];
    }

    std::cout << std::endl;
}
