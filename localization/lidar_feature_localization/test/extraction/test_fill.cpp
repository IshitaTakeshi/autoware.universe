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

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "lidar_feature_extraction/fill.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"


TEST(Label, FillFromLeft)
{
  {
    std::vector<PointLabel> labels = InitLabels(5);

    FillFromLeft(labels, 1, 4, PointLabel::Edge);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(5);

    FillFromLeft(labels, 1, 5, PointLabel::Edge);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Edge));
  }
}

TEST(Label, FillFromRight)
{
  {
    std::vector<PointLabel> labels = InitLabels(5);

    FillFromRight(labels, 0, 2, PointLabel::Edge);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(5);

    FillFromRight(labels, 1, 3, PointLabel::Edge);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(5);

    FillFromRight(labels, 1, 4, PointLabel::Edge);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Edge));
  }
}

TEST(Label, FillNeighbors)
{
  {
    std::vector<PointLabel> labels = InitLabels(6);

    FillNeighbors(labels, 3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor));
  }

  {
    std::vector<PointLabel> labels = InitLabels(8);

    FillNeighbors(labels, 3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(6);

    FillNeighbors(labels, 1, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(6);

    FillNeighbors(labels, 4, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor));
  }

  {
    std::vector<PointLabel> labels = InitLabels(6);

    FillNeighbors(labels, 2, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Default));
  }
}
