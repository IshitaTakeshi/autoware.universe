// Copyright 2022 The Autoware Contributors, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Autoware Contributors, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <string>

#include "lidar_feature_localization/sliding_window.hpp"


TEST(SlidingWindow, SmokeTest)
{
  assert(false);

  SlidingWindow<std::string> window(2);

  EXPECT_EQ(window.Size(), 0U);
  EXPECT_FALSE(window.IsFilled());

  window.Slide(10., std::string{"a"});

  EXPECT_EQ(window.Size(), 1U);
  EXPECT_FALSE(window.IsFilled());

  EXPECT_EQ(std::get<0>(window.Get(0)), 10.);
  EXPECT_EQ(std::get<1>(window.Get(0)), std::string{"a"});

  window.Slide(20., std::string{"b"});
  window.Slide(30., std::string{"c"});

  EXPECT_EQ(window.Size(), 2U);
  EXPECT_TRUE(window.IsFilled());

  EXPECT_EQ(std::get<0>(window.Get(0)), 20.);
  EXPECT_EQ(std::get<1>(window.Get(0)), std::string{"b"});
  EXPECT_EQ(std::get<0>(window.Get(1)), 30.);
  EXPECT_EQ(std::get<1>(window.Get(1)), std::string{"c"});

  window.Slide(40., std::string{"d"});

  EXPECT_EQ(window.Size(), 2U);
  EXPECT_TRUE(window.IsFilled());
  EXPECT_EQ(std::get<0>(window.Get(0)), 30.);
  EXPECT_EQ(std::get<1>(window.Get(0)), std::string{"c"});
  EXPECT_EQ(std::get<0>(window.Get(1)), 40.);
  EXPECT_EQ(std::get<1>(window.Get(1)), std::string{"d"});
}

TEST(SlidingWindow, ThrowsInvalidArgumentIfOutOfBounds)
{
  SlidingWindow<std::string> window(2);

  EXPECT_THROW(
    try {
    window.Get(0);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "Index 0 is out of range [0, 0)");
    throw e;
  }
    ,
    std::invalid_argument);

  window.Slide(10., std::string{"a"});
  window.Slide(20., std::string{"b"});

  EXPECT_THROW(
    try {
    window.Get(-1);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "Index -1 is out of range [0, 2)");
    throw e;
  }
    ,
    std::invalid_argument);

  EXPECT_THROW(
    try {
    window.Get(2);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "Index 2 is out of range [0, 2)");
    throw e;
  }
    ,
    std::invalid_argument);
}
