#include "gtest/gtest.h"
#include <iostream>

// Demonstrate some basic assertions.
TEST(HelloTestInLibrary, BasicAssertions) {
  {
    testing::internal::CaptureStdout();
    std::cout << "Hello, World!" << std::endl;
    testing::internal::GetCapturedStdout();
  }
  
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}
