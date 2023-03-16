// Copyright 2022 Tixiao Shan, Takeshi Ishita
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
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
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
#include <tuple>

#include "lidar_feature_localization/stamp_sorted_objects.hpp"


TEST(StampSortedObjects, GetClosest)
{
  StampSortedObjects<std::string> q;
  q.Insert(1.0, std::string{"a"});
  q.Insert(2.0, std::string{"b"});
  q.Insert(3.0, std::string{"c"});
  q.Insert(4.0, std::string{"d"});

  ASSERT_EQ(q.Size(), static_cast<size_t>(4));

  EXPECT_EQ(q.GetClosest(0.0), std::make_tuple(1.0, "a"));

  EXPECT_EQ(q.GetClosest(2.4), std::make_tuple(2.0, "b"));

  EXPECT_EQ(q.GetClosest(2.5), std::make_tuple(3.0, "c"));
  EXPECT_EQ(q.GetClosest(3.0), std::make_tuple(3.0, "c"));

  EXPECT_EQ(q.GetClosest(4.0), std::make_tuple(4.0, "d"));
  EXPECT_EQ(q.GetClosest(5.0), std::make_tuple(4.0, "d"));
}

TEST(StampSortedObjects, GetLast)
{
  StampSortedObjects<std::string> q;

  q.Insert(1.0, std::string{"a"});
  EXPECT_EQ(q.GetLast(), std::make_tuple(1.0, "a"));

  q.Insert(2.0, std::string{"b"});
  EXPECT_EQ(q.GetLast(), std::make_tuple(2.0, "b"));

  q.Insert(3.0, std::string{"c"});
  EXPECT_EQ(q.GetLast(), std::make_tuple(3.0, "c"));

  q.Insert(4.0, std::string{"d"});
  EXPECT_EQ(q.GetLast(), std::make_tuple(4.0, "d"));
}

TEST(StampSortedObjects, RemoveOlderThan)
{
  StampSortedObjects<std::string> q;
  q.Insert(1.0, std::string{"a"});
  q.Insert(2.0, std::string{"b"});
  q.Insert(3.0, std::string{"c"});
  q.Insert(4.0, std::string{"d"});

  q.RemoveOlderThan(4.0);

  ASSERT_EQ(q.Size(), static_cast<size_t>(4));  // nothing removed

  q.RemoveOlderThan(3.0);

  ASSERT_EQ(q.Size(), static_cast<size_t>(3));

  EXPECT_EQ(q.GetClosest(1.0), std::make_tuple(1.0, "a"));
  EXPECT_EQ(q.GetClosest(2.0), std::make_tuple(2.0, "b"));
  EXPECT_EQ(q.GetClosest(3.0), std::make_tuple(3.0, "c"));
}

TEST(StampSortedObjects, GetPrev)
{
  StampSortedObjects<std::string> q;
  q.Insert(1.0, std::string{"a"});
  q.Insert(2.0, std::string{"b"});
  q.Insert(3.0, std::string{"c"});
  q.Insert(4.0, std::string{"d"});

  EXPECT_EQ(std::get<1>(q.GetPrev(1.5)), std::string{"a"});
  EXPECT_EQ(std::get<1>(q.GetPrev(2.0)), std::string{"a"});
  EXPECT_EQ(std::get<1>(q.GetPrev(2.5)), std::string{"b"});
  EXPECT_EQ(std::get<1>(q.GetPrev(3.0)), std::string{"b"});
  EXPECT_EQ(std::get<1>(q.GetPrev(3.5)), std::string{"c"});
  EXPECT_EQ(std::get<1>(q.GetPrev(4.0)), std::string{"c"});
  EXPECT_EQ(std::get<1>(q.GetPrev(4.5)), std::string{"d"});

  EXPECT_THROW(
    try {
    q.GetPrev(1.0);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "There's no element before 1");
    throw e;
  }
    ,
    std::invalid_argument
  );
}

TEST(StampSortedObjects, GetNext)
{
  StampSortedObjects<std::string> q;
  q.Insert(1.0, std::string{"a"});
  q.Insert(2.0, std::string{"b"});
  q.Insert(3.0, std::string{"c"});
  q.Insert(4.0, std::string{"d"});

  EXPECT_EQ(std::get<1>(q.GetNext(0.8)), std::string{"a"});
  EXPECT_EQ(std::get<1>(q.GetNext(1.0)), std::string{"b"});
  EXPECT_EQ(std::get<1>(q.GetNext(1.1)), std::string{"b"});
  EXPECT_EQ(std::get<1>(q.GetNext(2.0)), std::string{"c"});
  EXPECT_EQ(std::get<1>(q.GetNext(2.9)), std::string{"c"});
  EXPECT_EQ(std::get<1>(q.GetNext(3.0)), std::string{"d"});
  EXPECT_EQ(std::get<1>(q.GetNext(3.5)), std::string{"d"});

  EXPECT_THROW(
    try {
    q.GetNext(4.0);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "There's no element after 4");
    throw e;
  }
    ,
    std::invalid_argument
  );
}