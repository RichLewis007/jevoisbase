/** @file
 *	@brief MAVLink comm testsuite protocol generated from jevoismavlink.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "jevoismavlink.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(jevoismavlink, HEARTBEAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.autopilot = 84;
    packet_in.base_mode = 151;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 218;
    packet_in.mavlink_version = 2;

    mavlink::jevoismavlink::msg::HEARTBEAT packet1{};
    mavlink::jevoismavlink::msg::HEARTBEAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.autopilot, packet2.autopilot);
    EXPECT_EQ(packet1.base_mode, packet2.base_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.system_status, packet2.system_status);
    EXPECT_EQ(packet1.mavlink_version, packet2.mavlink_version);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, HEARTBEAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_heartbeat_t packet_c {
         963497464, 17, 84, 151, 218, 2
    };

    mavlink::jevoismavlink::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.autopilot = 84;
    packet_in.base_mode = 151;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 218;
    packet_in.mavlink_version = 2;

    mavlink::jevoismavlink::msg::HEARTBEAT packet2{};

    mavlink_msg_heartbeat_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.autopilot, packet2.autopilot);
    EXPECT_EQ(packet_in.base_mode, packet2.base_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.system_status, packet2.system_status);
    EXPECT_EQ(packet_in.mavlink_version, packet2.mavlink_version);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, PING)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::jevoismavlink::msg::PING packet1{};
    mavlink::jevoismavlink::msg::PING packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, PING)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ping_t packet_c {
         93372036854775807ULL, 963497880, 41, 108
    };

    mavlink::jevoismavlink::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::jevoismavlink::msg::PING packet2{};

    mavlink_msg_ping_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, PARAM_REQUEST_READ)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::jevoismavlink::msg::PARAM_REQUEST_READ packet1{};
    mavlink::jevoismavlink::msg::PARAM_REQUEST_READ packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, PARAM_REQUEST_READ)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_read_t packet_c {
         17235, 139, 206, "EFGHIJKLMNOPQRS"
    };

    mavlink::jevoismavlink::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::jevoismavlink::msg::PARAM_REQUEST_READ packet2{};

    mavlink_msg_param_request_read_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, PARAM_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::jevoismavlink::msg::PARAM_REQUEST_LIST packet1{};
    mavlink::jevoismavlink::msg::PARAM_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, PARAM_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_list_t packet_c {
         5, 72
    };

    mavlink::jevoismavlink::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::jevoismavlink::msg::PARAM_REQUEST_LIST packet2{};

    mavlink_msg_param_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, PARAM_VALUE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::jevoismavlink::msg::PARAM_VALUE packet1{};
    mavlink::jevoismavlink::msg::PARAM_VALUE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_count, packet2.param_count);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, PARAM_VALUE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_value_t packet_c {
         17.0, 17443, 17547, "IJKLMNOPQRSTUVW", 77
    };

    mavlink::jevoismavlink::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::jevoismavlink::msg::PARAM_VALUE packet2{};

    mavlink_msg_param_value_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_count, packet2.param_count);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, PARAM_SET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::jevoismavlink::msg::PARAM_SET packet1{};
    mavlink::jevoismavlink::msg::PARAM_SET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, PARAM_SET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_set_t packet_c {
         17.0, 17, 84, "GHIJKLMNOPQRSTU", 199
    };

    mavlink::jevoismavlink::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::jevoismavlink::msg::PARAM_SET packet2{};

    mavlink_msg_param_set_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::jevoismavlink::msg::ATTITUDE packet1{};
    mavlink::jevoismavlink::msg::ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::jevoismavlink::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::jevoismavlink::msg::ATTITUDE packet2{};

    mavlink_msg_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, OPTICAL_FLOW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 77;
    packet_in.flow_x = 18275;
    packet_in.flow_y = 18379;
    packet_in.flow_comp_m_x = 73.0;
    packet_in.flow_comp_m_y = 101.0;
    packet_in.quality = 144;
    packet_in.ground_distance = 129.0;
    packet_in.flow_rate_x = 199.0;
    packet_in.flow_rate_y = 227.0;

    mavlink::jevoismavlink::msg::OPTICAL_FLOW packet1{};
    mavlink::jevoismavlink::msg::OPTICAL_FLOW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet1.flow_x, packet2.flow_x);
    EXPECT_EQ(packet1.flow_y, packet2.flow_y);
    EXPECT_EQ(packet1.flow_comp_m_x, packet2.flow_comp_m_x);
    EXPECT_EQ(packet1.flow_comp_m_y, packet2.flow_comp_m_y);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.ground_distance, packet2.ground_distance);
    EXPECT_EQ(packet1.flow_rate_x, packet2.flow_rate_x);
    EXPECT_EQ(packet1.flow_rate_y, packet2.flow_rate_y);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, OPTICAL_FLOW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_optical_flow_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 18275, 18379, 77, 144, 199.0, 227.0
    };

    mavlink::jevoismavlink::msg::OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 77;
    packet_in.flow_x = 18275;
    packet_in.flow_y = 18379;
    packet_in.flow_comp_m_x = 73.0;
    packet_in.flow_comp_m_y = 101.0;
    packet_in.quality = 144;
    packet_in.ground_distance = 129.0;
    packet_in.flow_rate_x = 199.0;
    packet_in.flow_rate_y = 227.0;

    mavlink::jevoismavlink::msg::OPTICAL_FLOW packet2{};

    mavlink_msg_optical_flow_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet_in.flow_x, packet2.flow_x);
    EXPECT_EQ(packet_in.flow_y, packet2.flow_y);
    EXPECT_EQ(packet_in.flow_comp_m_x, packet2.flow_comp_m_x);
    EXPECT_EQ(packet_in.flow_comp_m_y, packet2.flow_comp_m_y);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.ground_distance, packet2.ground_distance);
    EXPECT_EQ(packet_in.flow_rate_x, packet2.flow_rate_x);
    EXPECT_EQ(packet_in.flow_rate_y, packet2.flow_rate_y);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, VISION_POSITION_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;

    mavlink::jevoismavlink::msg::VISION_POSITION_ESTIMATE packet1{};
    mavlink::jevoismavlink::msg::VISION_POSITION_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, VISION_POSITION_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vision_position_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::jevoismavlink::msg::VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;

    mavlink::jevoismavlink::msg::VISION_POSITION_ESTIMATE packet2{};

    mavlink_msg_vision_position_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, VISION_SPEED_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::VISION_SPEED_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;

    mavlink::jevoismavlink::msg::VISION_SPEED_ESTIMATE packet1{};
    mavlink::jevoismavlink::msg::VISION_SPEED_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, VISION_SPEED_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vision_speed_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0
    };

    mavlink::jevoismavlink::msg::VISION_SPEED_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;

    mavlink::jevoismavlink::msg::VISION_SPEED_ESTIMATE packet2{};

    mavlink_msg_vision_speed_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, OPTICAL_FLOW_RAD)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::OPTICAL_FLOW_RAD packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::jevoismavlink::msg::OPTICAL_FLOW_RAD packet1{};
    mavlink::jevoismavlink::msg::OPTICAL_FLOW_RAD packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet1.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet1.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet1.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet1.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet1.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet1.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet1.distance, packet2.distance);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, OPTICAL_FLOW_RAD)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_optical_flow_rad_t packet_c {
         93372036854775807ULL, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 963499128, 269.0, 19315, 3, 70
    };

    mavlink::jevoismavlink::msg::OPTICAL_FLOW_RAD packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::jevoismavlink::msg::OPTICAL_FLOW_RAD packet2{};

    mavlink_msg_optical_flow_rad_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet_in.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet_in.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet_in.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet_in.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet_in.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet_in.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet_in.distance, packet2.distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, CAMERA_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 77;
    packet_in.camera_count = 144;
    packet_in.vendor_name = {{ 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }};
    packet_in.model_name = {{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82 }};
    packet_in.firmware_version = 963497672;
    packet_in.focal_length = 73.0;
    packet_in.sensor_size_h = 101.0;
    packet_in.sensor_size_v = 129.0;
    packet_in.resolution_h = 18275;
    packet_in.resolution_v = 18379;
    packet_in.lens_id = 147;

    mavlink::jevoismavlink::msg::CAMERA_INFORMATION packet1{};
    mavlink::jevoismavlink::msg::CAMERA_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.camera_count, packet2.camera_count);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet1.focal_length, packet2.focal_length);
    EXPECT_EQ(packet1.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet1.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet1.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet1.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet1.lens_id, packet2.lens_id);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, CAMERA_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_information_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 18275, 18379, 77, 144, { 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }, { 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82 }, 147
    };

    mavlink::jevoismavlink::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 77;
    packet_in.camera_count = 144;
    packet_in.vendor_name = {{ 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }};
    packet_in.model_name = {{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82 }};
    packet_in.firmware_version = 963497672;
    packet_in.focal_length = 73.0;
    packet_in.sensor_size_h = 101.0;
    packet_in.sensor_size_v = 129.0;
    packet_in.resolution_h = 18275;
    packet_in.resolution_v = 18379;
    packet_in.lens_id = 147;

    mavlink::jevoismavlink::msg::CAMERA_INFORMATION packet2{};

    mavlink_msg_camera_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.camera_count, packet2.camera_count);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet_in.focal_length, packet2.focal_length);
    EXPECT_EQ(packet_in.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet_in.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet_in.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet_in.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet_in.lens_id, packet2.lens_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(jevoismavlink, CAMERA_SETTINGS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::jevoismavlink::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 77;
    packet_in.exposure_mode = 144;
    packet_in.aperture = 45.0;
    packet_in.shutter_speed = 73.0;
    packet_in.iso_sensitivity = 101.0;
    packet_in.ev = 129.0;
    packet_in.white_balance = 157.0;
    packet_in.mode_id = 211;
    packet_in.audio_recording = 22;
    packet_in.color_mode_id = 89;
    packet_in.image_format_id = 156;
    packet_in.image_quality_id = 223;
    packet_in.metering_mode_id = 34;
    packet_in.flicker_mode_id = 101;

    mavlink::jevoismavlink::msg::CAMERA_SETTINGS packet1{};
    mavlink::jevoismavlink::msg::CAMERA_SETTINGS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.exposure_mode, packet2.exposure_mode);
    EXPECT_EQ(packet1.aperture, packet2.aperture);
    EXPECT_EQ(packet1.shutter_speed, packet2.shutter_speed);
    EXPECT_EQ(packet1.iso_sensitivity, packet2.iso_sensitivity);
    EXPECT_EQ(packet1.ev, packet2.ev);
    EXPECT_EQ(packet1.white_balance, packet2.white_balance);
    EXPECT_EQ(packet1.mode_id, packet2.mode_id);
    EXPECT_EQ(packet1.audio_recording, packet2.audio_recording);
    EXPECT_EQ(packet1.color_mode_id, packet2.color_mode_id);
    EXPECT_EQ(packet1.image_format_id, packet2.image_format_id);
    EXPECT_EQ(packet1.image_quality_id, packet2.image_quality_id);
    EXPECT_EQ(packet1.metering_mode_id, packet2.metering_mode_id);
    EXPECT_EQ(packet1.flicker_mode_id, packet2.flicker_mode_id);
}

#ifdef TEST_INTEROP
TEST(jevoismavlink_interop, CAMERA_SETTINGS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_settings_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 77, 144, 211, 22, 89, 156, 223, 34, 101
    };

    mavlink::jevoismavlink::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 77;
    packet_in.exposure_mode = 144;
    packet_in.aperture = 45.0;
    packet_in.shutter_speed = 73.0;
    packet_in.iso_sensitivity = 101.0;
    packet_in.ev = 129.0;
    packet_in.white_balance = 157.0;
    packet_in.mode_id = 211;
    packet_in.audio_recording = 22;
    packet_in.color_mode_id = 89;
    packet_in.image_format_id = 156;
    packet_in.image_quality_id = 223;
    packet_in.metering_mode_id = 34;
    packet_in.flicker_mode_id = 101;

    mavlink::jevoismavlink::msg::CAMERA_SETTINGS packet2{};

    mavlink_msg_camera_settings_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.exposure_mode, packet2.exposure_mode);
    EXPECT_EQ(packet_in.aperture, packet2.aperture);
    EXPECT_EQ(packet_in.shutter_speed, packet2.shutter_speed);
    EXPECT_EQ(packet_in.iso_sensitivity, packet2.iso_sensitivity);
    EXPECT_EQ(packet_in.ev, packet2.ev);
    EXPECT_EQ(packet_in.white_balance, packet2.white_balance);
    EXPECT_EQ(packet_in.mode_id, packet2.mode_id);
    EXPECT_EQ(packet_in.audio_recording, packet2.audio_recording);
    EXPECT_EQ(packet_in.color_mode_id, packet2.color_mode_id);
    EXPECT_EQ(packet_in.image_format_id, packet2.image_format_id);
    EXPECT_EQ(packet_in.image_quality_id, packet2.image_quality_id);
    EXPECT_EQ(packet_in.metering_mode_id, packet2.metering_mode_id);
    EXPECT_EQ(packet_in.flicker_mode_id, packet2.flicker_mode_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
