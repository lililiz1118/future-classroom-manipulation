#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <thread>

#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include "dh_device.h"

TEST(DhDeviceReadTest, AssemblesAResponseThatArrivesInMultipleChunks)
{
  int master_fd = -1;
  int slave_fd = -1;
  ASSERT_EQ(0, openpty(&master_fd, &slave_fd, nullptr, nullptr, nullptr));

  struct termios settings;
  ASSERT_EQ(0, tcgetattr(slave_fd, &settings));
  cfmakeraw(&settings);
  ASSERT_EQ(0, tcsetattr(slave_fd, TCSANOW, &settings));

  const std::array<unsigned char, 7> response = {
      0x01, 0x03, 0x02, 0x03, 0xE8, 0xB8, 0xFA};
  std::thread writer([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(2, write(master_fd, response.data(), 2));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_EQ(5, write(master_fd, response.data() + 2, 5));
  });

  std::array<char, 7> received = {};
  const int received_length =
      device_read(slave_fd, received.data(), received.size());
  writer.join();

  EXPECT_EQ(7, received_length);
  for (std::size_t index = 0; index < response.size(); ++index)
  {
    EXPECT_EQ(response[index], static_cast<unsigned char>(received[index]));
  }

  close(slave_fd);
  close(master_fd);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
