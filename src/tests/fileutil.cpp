
#include "tests/gtest/gtest.h"
#include "fileutil.h"

TEST(FileUtil, HasExtension) {
    EXPECT_TRUE(HasExtension("foo.exr", "exr"));
    EXPECT_TRUE(HasExtension("foo.Exr", "exr"));
    EXPECT_TRUE(HasExtension("foo.Exr", "exR"));
    EXPECT_TRUE(HasExtension("foo.EXR", "exr"));
    EXPECT_FALSE(HasExtension("foo.xr", "exr"));
    EXPECT_FALSE(HasExtension("/foo/png", "ppm"));
}


TEST(FileUtil, AbsolutePath) {
#ifdef PBRT_IS_WINDOWS
    EXPECT_TRUE(IsAbsolutePath("\\\\foo\\bar.exe"));
    EXPECT_TRUE(IsAbsolutePath("c:\\foo\\bar.exe"));
    EXPECT_FALSE(IsAbsolutePath("foo\\bar"));
#endif // PBRT_IS_WINDOWS
    EXPECT_TRUE(IsAbsolutePath("/foo/bar"));
    EXPECT_FALSE(IsAbsolutePath("foo/bar"));
}
