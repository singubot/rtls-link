#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include <rtlslink/mavlink.h>

namespace {

constexpr std::array<int32_t, 8> kX = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
constexpr std::array<int32_t, 8> kY = {9000, 10000, 11000, 12000, 13000, 14000, 15000, 16000};
constexpr std::array<int32_t, 8> kZ = {-1000, -2000, -3000, -4000, -5000, -6000, -7000, -8000};
constexpr std::array<uint8_t, 8> kIds = {0, 1, 2, 3, 4, 5, 6, 7};
constexpr std::array<uint8_t, 4> kIp = {192, 168, 0, 104};
constexpr std::array<uint8_t, 6> kMac = {1, 2, 3, 4, 5, 6};

void ExpectDynamicAnchorArrays(const mavlink_message_t& message)
{
    mavlink_rtls_device_status_t status = {};
    mavlink_msg_rtls_device_status_decode(&message, &status);

    EXPECT_EQ(message.len, MAVLINK_MSG_ID_RTLS_DEVICE_STATUS_LEN);
    EXPECT_EQ(MAVLINK_MSG_ID_RTLS_DEVICE_STATUS_LEN, 175);
    EXPECT_EQ(MAVLINK_MSG_ID_RTLS_DEVICE_STATUS_MIN_LEN, 175);
    EXPECT_EQ(MAVLINK_MSG_ID_RTLS_DEVICE_STATUS_CRC, 247);

    for (size_t i = 0; i < kX.size(); ++i) {
        EXPECT_EQ(status.dynamic_anchor_id[i], kIds[i]);
        EXPECT_EQ(status.dynamic_anchor_x_mm[i], kX[i]);
        EXPECT_EQ(status.dynamic_anchor_y_mm[i], kY[i]);
        EXPECT_EQ(status.dynamic_anchor_z_mm[i], kZ[i]);
    }
}

}  // namespace

TEST(MavlinkRtlsDeviceStatus, PacksDynamicAnchorCoordinateArrays)
{
    mavlink_message_t message = {};

    mavlink_msg_rtls_device_status_pack(1,
                                        1,
                                        &message,
                                        123,
                                        0,
                                        1,
                                        2,
                                        3,
                                        3334,
                                        4,
                                        4,
                                        8,
                                        3,
                                        4,
                                        kIds.data(),
                                        kX.data(),
                                        kY.data(),
                                        kZ.data(),
                                        kIp.data(),
                                        kMac.data(),
                                        "rtls-link",
                                        "1",
                                        "1.0.0");

    ExpectDynamicAnchorArrays(message);
}

TEST(MavlinkRtlsDeviceStatus, PacksDynamicAnchorCoordinateArraysOnChannel)
{
    mavlink_message_t message = {};

    mavlink_msg_rtls_device_status_pack_chan(1,
                                             1,
                                             MAVLINK_COMM_0,
                                             &message,
                                             123,
                                             0,
                                             1,
                                             2,
                                             3,
                                             3334,
                                             4,
                                             4,
                                             8,
                                             3,
                                             4,
                                             kIds.data(),
                                             kX.data(),
                                             kY.data(),
                                             kZ.data(),
                                             kIp.data(),
                                             kMac.data(),
                                             "rtls-link",
                                             "1",
                                             "1.0.0");

    ExpectDynamicAnchorArrays(message);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
