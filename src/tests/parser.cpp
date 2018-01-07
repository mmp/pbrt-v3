
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "parser.h"

#include <fstream>
#include <initializer_list>
#include <string>
#include <vector>

using namespace pbrt;

static std::string inTestDir(const std::string &path) { return path; }

static std::vector<std::string> extract(Tokenizer *t) {
    std::vector<std::string> tokens;
    while (true) {
        string_view s = t->Next();
        if (s.empty())
            return tokens;
        tokens.push_back(std::string(s.data(), s.size()));
    }
}

static void checkTokens(Tokenizer *t,
                        std::initializer_list<std::string> expected) {
    std::vector<std::string> tokens = extract(t);
    auto iter = expected.begin();
    for (const std::string &s : tokens) {
        EXPECT_TRUE(iter != expected.end());
        EXPECT_EQ(*iter, s);
        ++iter;
    }
    EXPECT_TRUE(iter == expected.end());
}

TEST(Parser, TokenizerBasics) {
    std::vector<std::string> errors;
    auto err = [&](const char *err) { errors.push_back(err); };

    {
        auto t = Tokenizer::CreateFromString("Shape \"sphere\" \"float radius\" [1]", err);
        ASSERT_TRUE(t.get() != nullptr);
        checkTokens(t.get(), {"Shape", "\"sphere\"", "\"float radius\"", "[", "1", "]"});
    }

    {
        auto t = Tokenizer::CreateFromString("Shape \"sphere\"\n\"float radius\" [1]", err);
        ASSERT_TRUE(t.get() != nullptr);
        checkTokens(t.get(), {"Shape", "\"sphere\"", "\"float radius\"", "[", "1", "]"});
    }

    {
        auto t = Tokenizer::CreateFromString(R"(
Shape"sphere" # foo bar [
"float radius\"" 1)", err);
        ASSERT_TRUE(t.get() != nullptr);
        checkTokens(t.get(), {"Shape", "\"sphere\"", "# foo bar [", R"("float radius"")", "1"});
    }
}

TEST(Parser, TokenizerErrors) {
    {
        bool gotError = false;
        auto err = [&](const char *err) {
            gotError = !strcmp(err, "premature EOF");
        };
        auto t = Tokenizer::CreateFromString("Shape\"sphere\"\t\t # foo bar\n\"float radius", err);
        ASSERT_TRUE(t.get() != nullptr);
        extract(t.get());
        EXPECT_TRUE(gotError);
    }

    {
        bool gotError = false;
        auto err = [&](const char *err) {
            gotError = !strcmp(err, "premature EOF");
        };
        auto t = Tokenizer::CreateFromString("Shape\"sphere\"\t\t # foo bar\n\"float radius", err);
        ASSERT_TRUE(t.get() != nullptr);
        extract(t.get());
        EXPECT_TRUE(gotError);
    }

    {
        bool gotError = false;
        auto err = [&](const char *err) {
            gotError = !strcmp(err, "premature EOF");
        };
        auto t = Tokenizer::CreateFromString("Shape\"sphere\"\t\t # foo bar\n\"float radius\\", err);
        ASSERT_TRUE(t.get() != nullptr);
        extract(t.get());
        EXPECT_TRUE(gotError);
    }

    {
        bool gotError = false;
        auto err = [&](const char *err) {
            gotError = !strcmp(err, "unterminated string");
        };
        auto t = Tokenizer::CreateFromString("Shape\"sphere\"\t\t # foo bar\n\"float radius\n\" 5", err);
        ASSERT_TRUE(t.get() != nullptr);
        extract(t.get());
        EXPECT_TRUE(gotError);
    }
}

TEST(Parser, TokenizeFile) {
    std::string filename = inTestDir("test.tok");
    std::ofstream out(filename);
    out << R"(
WorldBegin # hello
Integrator "deep" "float density" [ 2 2.66612 -5e-51]
)";
    out.close();
    ASSERT_TRUE(out.good());

    auto err = [](const char *err) {
        EXPECT_TRUE(false) << "Unexpected error: " << err;
    };
    auto t = Tokenizer::CreateFromFile(filename, err);
    ASSERT_TRUE(t.get() != nullptr);
    checkTokens(t.get(), {"WorldBegin", "# hello", "Integrator", "\"deep\"", "\"float density\"",
                          "[", "2", "2.66612", "-5e-51", "]"});

    EXPECT_EQ(0, remove(filename.c_str()));
}

