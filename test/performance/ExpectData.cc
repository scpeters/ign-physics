/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <chrono>
#include <iomanip>
#include <cmath>

#include <gtest/gtest.h>

#include "utils/TestDataTypes.hh"

#define IGN_PHYSICS_CREATE_LABELED_DATA(Name) \
  struct Name { IGN_PHYSICS_DATA_LABEL(Name) };

IGN_PHYSICS_CREATE_LABELED_DATA(SomeData1)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData2)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData3)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData4)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData5)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData6)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData7)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData8)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData9)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData10)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData11)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData12)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData13)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData14)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData15)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData16)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData17)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData18)
IGN_PHYSICS_CREATE_LABELED_DATA(SomeData19)

// Expect only 1 type
using ExpectString = ignition::physics::ExpectData<StringData>;

// Expect 3 different types, and put the type we care about first in the list
using Expect3Types_Leading =
    ignition::physics::ExpectData<StringData, BoolData, CharData>;

// Expect 3 different types, and put the type we care about last in the list
using Expect3Types_Trailing =
    ignition::physics::ExpectData<CharData, BoolData, StringData>;

// Expect 10 different types, and put the type we care about first in the list
using Expect10Types_Leading =
    ignition::physics::ExpectData<
        StringData,
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9>;

// Expect 10 different types, and put the type we care about last in the list
using Expect10Types_Trailing =
    ignition::physics::ExpectData<
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9,
        StringData>;

// Expect 20 different types, and put the type we care about first in the list
using Expect20Types_Leading =
    ignition::physics::ExpectData<
        StringData,
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9, SomeData10,
        SomeData11, SomeData12, SomeData13, SomeData14, SomeData15,
        SomeData16, SomeData17, SomeData18, SomeData19>;

// Expect 20 different types, and put the type we care about last in the list
using Expect20Types_Trailing =
    ignition::physics::ExpectData<
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9, SomeData10,
        SomeData11, SomeData12, SomeData13, SomeData14, SomeData15,
        SomeData16, SomeData17, SomeData18, SomeData19,
        StringData>;

ignition::physics::CompositeData CreatePerformanceTestData()
{
  return CreateSomeData<StringData, DoubleData, IntData,
      FloatData, VectorDoubleData, BoolData, CharData>();
}

template <typename CompositeType>
double RunPerformanceTest(CompositeType &data)
{
  const std::size_t NumTests = 100000;
  const auto start = std::chrono::high_resolution_clock::now();
  for (std::size_t i=0; i < NumTests; ++i)
  {
    data.template Get<StringData>();
  }
  const auto finish = std::chrono::high_resolution_clock::now();

  const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        finish - start).count();

  const double avg = static_cast<double>(time)/static_cast<double>(NumTests);

  return avg;
}

TEST(ExpectData, AccessTime)
{
  ExpectString expect_1;
  Expect3Types_Leading expect_3_leading;
  Expect3Types_Trailing expect_3_trailing;
  Expect10Types_Leading expect_10_leading;
  Expect10Types_Trailing expect_10_trailing;
  Expect20Types_Leading expect_20_leading;
  Expect20Types_Trailing expect_20_trailing;
  ignition::physics::CompositeData plain;

  expect_1.Copy(CreatePerformanceTestData());
  expect_3_leading.Copy(CreatePerformanceTestData());
  expect_3_trailing.Copy(CreatePerformanceTestData());
  expect_10_leading.Copy(CreatePerformanceTestData());
  expect_10_trailing.Copy(CreatePerformanceTestData());
  expect_20_leading.Copy(CreatePerformanceTestData());
  expect_20_trailing.Copy(CreatePerformanceTestData());
  plain.Copy(CreatePerformanceTestData());

  double avg_expect_1 = 0.0;
  double avg_expect_3_leading = 0.0;
  double avg_expect_3_trailing = 0.0;
  double avg_expect_10_leading = 0.0;
  double avg_expect_10_trailing = 0.0;
  double avg_expect_20_leading = 0.0;
  double avg_expect_20_trailing = 0.0;
  double avg_plain = 0.0;

  const std::size_t NumRuns = 100;

  for(std::size_t i=0; i < NumRuns; ++i)
  {
    avg_expect_1 += RunPerformanceTest(expect_1);
    avg_expect_3_leading += RunPerformanceTest(expect_3_leading);
    avg_expect_3_trailing += RunPerformanceTest(expect_3_trailing);
    avg_expect_10_leading += RunPerformanceTest(expect_10_leading);
    avg_expect_10_trailing += RunPerformanceTest(expect_10_trailing);
    avg_expect_20_leading += RunPerformanceTest(expect_20_leading);
    avg_expect_20_trailing += RunPerformanceTest(expect_20_trailing);
    avg_plain += RunPerformanceTest(plain);
  }

  EXPECT_LT(avg_expect_1, avg_plain);
  EXPECT_LT(avg_expect_3_leading, avg_plain);
  EXPECT_LT(avg_expect_3_trailing, avg_plain);
  EXPECT_LT(avg_expect_10_leading, avg_plain);
  EXPECT_LT(avg_expect_10_trailing, avg_plain);
  EXPECT_LT(avg_expect_20_leading, avg_plain);
  EXPECT_LT(avg_expect_20_trailing, avg_plain);

  // These 6 results should be very close to each other. avg_expect_1 is an
  // exception because it uses the plain ExpectData<T> instead of being wrapped
  // in a SpecifyData<T> which adds just a tiny bit of overhead because it needs
  // to make one additional function call.
  const double baseline = avg_expect_3_leading;
  EXPECT_LT(std::abs(avg_expect_3_trailing - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_10_leading - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_10_trailing - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_20_leading - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_20_trailing - baseline), baseline);

  std::vector<std::string> labels = {
    "1 expectation",
    "3 expectations (leading)",
    "3 expectations (ending)",
    "10 expectations (leading)",
    "10 expectations (trailing)",
    "20 expectations (leading)",
    "20 expectations (trailing)",
    "No expectations" };

  std::vector<double> avgs = {
    avg_expect_1,
    avg_expect_3_leading,
    avg_expect_3_trailing,
    avg_expect_10_leading,
    avg_expect_10_trailing,
    avg_expect_20_leading,
    avg_expect_20_trailing,
    avg_plain };


  for(std::size_t i=0; i < labels.size(); ++i)
  {
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << std::right;

    std::cout << " --- " << labels[i] << " result ---\n"
              << "Avg time: " << std::setw(8) << std::right
              << avgs[i]/static_cast<double>(NumRuns)
              << " ns\n" << std::endl;
  }
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
