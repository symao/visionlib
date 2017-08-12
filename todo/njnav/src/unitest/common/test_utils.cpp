#include <gtest/gtest.h>
#include <common/utils.h>

using namespace NJRobot;

TEST(common, FindKth)
{
	std::vector<int> a = { 1, 6, 8, 4, 9, 5, 2, 3, 7, 0 };
	ASSERT_EQ(findKth(a, 3), 3);
	ASSERT_EQ(findKth(a, 8), 8);
	ASSERT_EQ(findKth(a, 0), 0);
	ASSERT_EQ(findKth(a, 9), 9);
	ASSERT_EQ(findKth(a, -1), 0);
	ASSERT_EQ(findKth(a, 11), 9);
	ASSERT_EQ(vecMin(a), 0);
	ASSERT_EQ(vecMax(a), 9);
	ASSERT_FALSE(isAllZero(a));
}