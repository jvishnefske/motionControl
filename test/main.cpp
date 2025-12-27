//
// Created by cnh on 5/7/23.
// Unit tests for motion control networking library
//

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <sstream>
#include <iterator>
#include <numeric>
#include <string>
#include <chrono>
#include <functional>

// Test fixture for motion data calculations
class MotionDataTest : public ::testing::Test {
protected:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr int DEFAULT_FPS = 100;

    double calculateDelta(int fps) {
        return 2 * PI / fps;
    }

    std::pair<double, double> calculateSinCos(int frame, double delta) {
        return {std::sin(frame * delta), std::cos(frame * delta)};
    }
};

// Test delta calculation for different FPS values
TEST_F(MotionDataTest, DeltaCalculationAt100FPS) {
    double delta = calculateDelta(100);
    double expected = 2 * PI / 100;
    EXPECT_DOUBLE_EQ(delta, expected);
}

TEST_F(MotionDataTest, DeltaCalculationAt60FPS) {
    double delta = calculateDelta(60);
    double expected = 2 * PI / 60;
    EXPECT_DOUBLE_EQ(delta, expected);
}

TEST_F(MotionDataTest, DeltaCalculationAt30FPS) {
    double delta = calculateDelta(30);
    double expected = 2 * PI / 30;
    EXPECT_DOUBLE_EQ(delta, expected);
}

// Test sin/cos motion pattern generation
TEST_F(MotionDataTest, SinCosAtFrameZero) {
    double delta = calculateDelta(100);
    auto [sin_val, cos_val] = calculateSinCos(0, delta);

    EXPECT_NEAR(sin_val, 0.0, 1e-10);
    EXPECT_NEAR(cos_val, 1.0, 1e-10);
}

TEST_F(MotionDataTest, SinCosAtQuarterPeriod) {
    int fps = 100;
    double delta = calculateDelta(fps);
    int quarter_period_frame = fps / 4;  // 25 frames = 90 degrees
    auto [sin_val, cos_val] = calculateSinCos(quarter_period_frame, delta);

    EXPECT_NEAR(sin_val, 1.0, 1e-10);
    EXPECT_NEAR(cos_val, 0.0, 1e-10);
}

TEST_F(MotionDataTest, SinCosAtHalfPeriod) {
    int fps = 100;
    double delta = calculateDelta(fps);
    int half_period_frame = fps / 2;  // 50 frames = 180 degrees
    auto [sin_val, cos_val] = calculateSinCos(half_period_frame, delta);

    EXPECT_NEAR(sin_val, 0.0, 1e-10);
    EXPECT_NEAR(cos_val, -1.0, 1e-10);
}

TEST_F(MotionDataTest, SinCosAtFullPeriod) {
    int fps = 100;
    double delta = calculateDelta(fps);
    auto [sin_val, cos_val] = calculateSinCos(fps, delta);

    EXPECT_NEAR(sin_val, 0.0, 1e-10);
    EXPECT_NEAR(cos_val, 1.0, 1e-10);
}

// Test motion values stay within valid range [-1, 1]
TEST_F(MotionDataTest, MotionValuesWithinBounds) {
    int fps = 100;
    double delta = calculateDelta(fps);

    for (int frame = 0; frame < fps * 2; ++frame) {
        auto [sin_val, cos_val] = calculateSinCos(frame, delta);
        EXPECT_GE(sin_val, -1.0) << "Sin value below -1 at frame " << frame;
        EXPECT_LE(sin_val, 1.0) << "Sin value above 1 at frame " << frame;
        EXPECT_GE(cos_val, -1.0) << "Cos value below -1 at frame " << frame;
        EXPECT_LE(cos_val, 1.0) << "Cos value above 1 at frame " << frame;
    }
}

// Test unit circle property: sin^2 + cos^2 = 1
TEST_F(MotionDataTest, UnitCircleProperty) {
    int fps = 100;
    double delta = calculateDelta(fps);

    for (int frame = 0; frame < fps; ++frame) {
        auto [sin_val, cos_val] = calculateSinCos(frame, delta);
        double sum_of_squares = sin_val * sin_val + cos_val * cos_val;
        EXPECT_NEAR(sum_of_squares, 1.0, 1e-10)
            << "Unit circle property violated at frame " << frame;
    }
}

// Test fixture for vector serialization (similar to send_vector logic)
class VectorSerializationTest : public ::testing::Test {
protected:
    // Simulates the partial_sum serialization used in send_vector
    std::string serializeVector(const std::vector<double>& numbers) {
        std::ostringstream oss;
        oss << "put command here";
        std::partial_sum(std::begin(numbers), std::end(numbers),
                        std::ostream_iterator<int>(oss, " "));
        return oss.str();
    }
};

TEST_F(VectorSerializationTest, EmptyVector) {
    std::vector<double> empty;
    std::string result = serializeVector(empty);
    EXPECT_EQ(result, "put command here");
}

TEST_F(VectorSerializationTest, SingleElement) {
    std::vector<double> single{5.0};
    std::string result = serializeVector(single);
    EXPECT_EQ(result, "put command here5 ");
}

TEST_F(VectorSerializationTest, TwoElements) {
    std::vector<double> two{3.0, 4.0};
    std::string result = serializeVector(two);
    // partial_sum: [3, 3+4=7] -> "3 7 "
    EXPECT_EQ(result, "put command here3 7 ");
}

TEST_F(VectorSerializationTest, MultipleElements) {
    std::vector<double> nums{1.0, 2.0, 3.0, 4.0};
    std::string result = serializeVector(nums);
    // partial_sum: [1, 1+2=3, 3+3=6, 6+4=10] -> "1 3 6 10 "
    EXPECT_EQ(result, "put command here1 3 6 10 ");
}

TEST_F(VectorSerializationTest, NegativeValues) {
    std::vector<double> nums{-1.0, 2.0, -3.0};
    std::string result = serializeVector(nums);
    // partial_sum: [-1, -1+2=1, 1+(-3)=-2] -> "-1 1 -2 "
    EXPECT_EQ(result, "put command here-1 1 -2 ");
}

TEST_F(VectorSerializationTest, FloatingPointTruncation) {
    // Values are truncated to int in output
    std::vector<double> nums{1.9, 2.1};
    std::string result = serializeVector(nums);
    // 1.9 -> 1, 1+2.1 -> 3.0 -> 3
    // Note: partial_sum operates on doubles, then outputs as int
    EXPECT_EQ(result, "put command here1 4 ");
}

// Test fixture for ID message formatting
class IdMessageTest : public ::testing::Test {
protected:
    std::string formatIdMessage(const std::string& game_id) {
        return "#id " + game_id + "\n";
    }
};

TEST_F(IdMessageTest, SimpleGameId) {
    std::string result = formatIdMessage("myGame1");
    EXPECT_EQ(result, "#id myGame1\n");
}

TEST_F(IdMessageTest, EmptyGameId) {
    std::string result = formatIdMessage("");
    EXPECT_EQ(result, "#id \n");
}

TEST_F(IdMessageTest, GameIdWithSpaces) {
    std::string result = formatIdMessage("my game name");
    EXPECT_EQ(result, "#id my game name\n");
}

TEST_F(IdMessageTest, LongGameId) {
    std::string long_id = std::string(100, 'x');
    std::string result = formatIdMessage(long_id);
    EXPECT_EQ(result, "#id " + long_id + "\n");
}

// Test fixture for endpoint configuration
class EndpointConfigTest : public ::testing::Test {
protected:
    struct EndpointConfig {
        std::string ip;
        int port;
    };

    bool isValidPort(int port) {
        return port > 0 && port <= 65535;
    }

    bool isValidIPv4(const std::string& ip) {
        // Basic validation - check format
        int dots = 0;
        for (char c : ip) {
            if (c == '.') dots++;
            else if (!std::isdigit(c)) return false;
        }
        return dots == 3;
    }
};

TEST_F(EndpointConfigTest, DefaultPort) {
    EXPECT_TRUE(isValidPort(1299));  // Default port in UdpClient
}

TEST_F(EndpointConfigTest, ValidPortRange) {
    EXPECT_TRUE(isValidPort(1));
    EXPECT_TRUE(isValidPort(1024));
    EXPECT_TRUE(isValidPort(8080));
    EXPECT_TRUE(isValidPort(65535));
}

TEST_F(EndpointConfigTest, InvalidPortRange) {
    EXPECT_FALSE(isValidPort(0));
    EXPECT_FALSE(isValidPort(-1));
    EXPECT_FALSE(isValidPort(65536));
    EXPECT_FALSE(isValidPort(100000));
}

TEST_F(EndpointConfigTest, ValidIPv4Format) {
    EXPECT_TRUE(isValidIPv4("127.0.0.1"));
    EXPECT_TRUE(isValidIPv4("192.168.1.1"));
    EXPECT_TRUE(isValidIPv4("0.0.0.0"));
    EXPECT_TRUE(isValidIPv4("255.255.255.255"));
}

TEST_F(EndpointConfigTest, InvalidIPv4Format) {
    EXPECT_FALSE(isValidIPv4("localhost"));
    EXPECT_FALSE(isValidIPv4("abc.def.ghi.jkl"));
    EXPECT_FALSE(isValidIPv4("192.168.1"));
    EXPECT_FALSE(isValidIPv4(""));
}

// Test callback handler functionality
class PacketHandlerTest : public ::testing::Test {
protected:
    using PacketHandler = std::function<void(const std::string&)>;
};

TEST_F(PacketHandlerTest, HandlerCalled) {
    bool handler_called = false;
    std::string received_packet;

    PacketHandler handler = [&](const std::string& packet) {
        handler_called = true;
        received_packet = packet;
    };

    handler("test packet");

    EXPECT_TRUE(handler_called);
    EXPECT_EQ(received_packet, "test packet");
}

TEST_F(PacketHandlerTest, HandlerCalledMultipleTimes) {
    int call_count = 0;

    PacketHandler handler = [&](const std::string&) {
        call_count++;
    };

    handler("packet1");
    handler("packet2");
    handler("packet3");

    EXPECT_EQ(call_count, 3);
}

TEST_F(PacketHandlerTest, HandlerWithEmptyPacket) {
    std::string received;

    PacketHandler handler = [&](const std::string& packet) {
        received = packet;
    };

    handler("");

    EXPECT_TRUE(received.empty());
}

// Test timing calculations
class TimingTest : public ::testing::Test {
protected:
    int calculateFrameDelay(int fps) {
        return 1000 / fps;  // milliseconds per frame
    }
};

TEST_F(TimingTest, DelayAt100FPS) {
    EXPECT_EQ(calculateFrameDelay(100), 10);
}

TEST_F(TimingTest, DelayAt60FPS) {
    EXPECT_EQ(calculateFrameDelay(60), 16);
}

TEST_F(TimingTest, DelayAt30FPS) {
    EXPECT_EQ(calculateFrameDelay(30), 33);
}

// Main function provided by gtest_main
