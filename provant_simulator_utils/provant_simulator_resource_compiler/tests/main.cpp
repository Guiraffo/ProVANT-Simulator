#include <gtest/gtest.h>

#include <cmrc/cmrc.hpp>

CMRC_DECLARE(testrcs);

TEST(CMakeRC, ReadFromResourceFile)
{
  auto fs = cmrc::testrcs::get_filesystem();
  auto test1File = fs.open("tests/resources/test1.txt");

  const std::string str{ test1File.begin(), test1File.end() };

  ASSERT_EQ(std::string{ "Almost before we knew it, we had left the ground." }, str);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  try
  {
    return RUN_ALL_TESTS();
  }
  catch (const std::exception& e)
  {
    std::cerr << "An unexpected expection with message \"" << e.what() << "\" ocurred while running the tests.";
    return -1;
  }
}
