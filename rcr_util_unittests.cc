#include "gtest/gtest.h"

#include "rcr_util.hh"

namespace {

TEST(abs, abstest) {
  using rcr::util;
  EXPECT_EQ(abs(3), 3);
  EXPECT_EQ(abs(-3), 3);
  EXPECT_EQ(abs(10.), 10.);
  EXPECT_EQ(abs(-10.), 10.);
}

}  // namespace
