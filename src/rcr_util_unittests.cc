#include <gtest/gtest.h>

#include "rcr_util.hh"

namespace {

TEST(abs, abstest) {
  using namespace rcr::util;
  EXPECT_EQ(abs(3), 3);
  EXPECT_EQ(abs(-3), 3);
  EXPECT_EQ(abs(10.), 10.);
  EXPECT_EQ(abs(-10.), 10.);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
