#include <limits>

#include "check.hpp"
#include "routeplan/json.hpp"

using namespace routeplan::json;

TEST(parses_scalars) {
    CHECK(parse("null").is_null());
    CHECK_EQ(parse("true").as_bool(), true);
    CHECK_EQ(parse("false").as_bool(), false);
    CHECK_EQ(parse("12.5").as_number(), 12.5);
    CHECK_EQ(parse("-3e2").as_number(), -300.0);
    CHECK_EQ(parse("\"hi\"").as_string(), std::string("hi"));
}

TEST(parses_nested_structures_in_order) {
    Value value = parse(R"( {"b": [1, {"c": null}], "a": {}} )");
    CHECK_EQ(value.as_object().size(), std::size_t{2});
    CHECK_EQ(value.as_object()[0].first, std::string("b"));
    const Value* b = value.find("b");
    CHECK(b != nullptr);
    CHECK(b->as_array()[1].find("c")->is_null());
    CHECK(value.find("missing") == nullptr);
}

TEST(reads_escapes) {
    CHECK_EQ(parse(R"("a\n\t\"\\\/")").as_string(), std::string("a\n\t\"\\/"));
    CHECK_EQ(parse(R"("\u00e9")").as_string(), std::string("\xC3\xA9"));
    CHECK_EQ(parse(R"("\u20ac")").as_string(), std::string("\xE2\x82\xAC"));
    CHECK_EQ(parse(R"("\ud83d\ude00")").as_string(), std::string("\xF0\x9F\x98\x80"));
}

TEST(rejects_lone_surrogates) {
    CHECK_THROWS(parse(R"("\ud83d")"), ParseError);
    CHECK_THROWS(parse(R"("\ud83dx")"), ParseError);
    CHECK_THROWS(parse(R"("\ude00")"), ParseError);
    CHECK_THROWS(parse(R"("\ud83d\u0041")"), ParseError);
}

TEST(rejects_malformed_numbers) {
    for (const char* text : {"01", "1.", ".5", "+1", "1e", "-", "1e+", "NaN", "Infinity"}) {
        CHECK_THROWS(parse(text), ParseError);
    }
}

TEST(rejects_malformed_structures) {
    for (const char* text : {"", "[1,]", "{\"a\":1,}", "{\"a\"}", "{a:1}", "[1 2]", "tru", "\"open", "[1] 2", "\"a\nb\"", "\"\\q\""}) {
        CHECK_THROWS(parse(text), ParseError);
    }
}

TEST(rejects_duplicate_keys) {
    CHECK_THROWS(parse(R"({"a":1,"a":2})"), ParseError);
}

TEST(errors_carry_the_position) {
    try {
        parse("{\n  \"a\": ?\n}");
        CHECK(false);
    } catch (const ParseError& e) {
        CHECK_EQ(e.line(), 2);
        CHECK_EQ(e.column(), 8);
    }
}

TEST(rejects_very_deep_nesting) {
    std::string deep(500, '[');
    deep += std::string(500, ']');
    CHECK_THROWS(parse(deep), ParseError);
    std::string shallow(100, '[');
    shallow += std::string(100, ']');
    CHECK(parse(shallow).is_array());
}

TEST(wrong_type_access_throws) {
    Value value = parse("1");
    CHECK_THROWS(value.as_string(), TypeError);
    CHECK_THROWS(value.as_array(), TypeError);
    CHECK_THROWS(value.as_object(), TypeError);
    CHECK_THROWS(value.as_bool(), TypeError);
    CHECK_THROWS(value.find("a"), TypeError);
    CHECK_THROWS(parse("\"a\"").as_number(), TypeError);
}

TEST(dumps_compact_and_round_trips) {
    std::string text = R"({"a":[1,2.5,"x\n",null,true],"b":{}})";
    CHECK_EQ(dump(parse(text)), text);
}

TEST(dumps_pretty) {
    CHECK_EQ(dump(parse(R"({"a":[1],"b":[]})"), 2), std::string("{\n  \"a\": [\n    1\n  ],\n  \"b\": []\n}"));
}

TEST(dumps_whole_numbers_without_a_point) {
    CHECK_EQ(dump(Value(3.0)), std::string("3"));
    CHECK_EQ(dump(Value(-40.0)), std::string("-40"));
    CHECK_EQ(dump(Value(0.1)), std::string("0.1"));
}

TEST(dumps_infinity_as_null) {
    CHECK_EQ(dump(Value(std::numeric_limits<double>::infinity())), std::string("null"));
}

TEST(dumps_control_characters) {
    CHECK_EQ(dump(Value(std::string("a\x01"))), std::string("\"a\\u0001\""));
}
