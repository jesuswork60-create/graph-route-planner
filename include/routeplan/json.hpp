// A small JSON reader and writer, just enough for graph files and machine-readable output.
#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace routeplan::json {

class ParseError : public std::runtime_error {
public:
    ParseError(const std::string& message, int line, int column);
    int line() const { return line_; }
    int column() const { return column_; }

private:
    int line_;
    int column_;
};

class TypeError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Value {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Value>;
    using Member = std::pair<std::string, Value>;
    using Object = std::vector<Member>;

    Value() = default;
    Value(std::nullptr_t) {}
    Value(bool value) : type_(Type::Bool), bool_(value) {}
    Value(double value) : type_(Type::Number), number_(value) {}
    Value(int value) : type_(Type::Number), number_(value) {}
    Value(const char* value) : type_(Type::String), string_(value) {}
    Value(std::string value) : type_(Type::String), string_(std::move(value)) {}
    Value(Array value) : type_(Type::Array), array_(std::move(value)) {}
    Value(Object value) : type_(Type::Object), object_(std::move(value)) {}

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }
    bool is_object() const { return type_ == Type::Object; }
    bool is_array() const { return type_ == Type::Array; }
    bool is_string() const { return type_ == Type::String; }
    bool is_number() const { return type_ == Type::Number; }

    bool as_bool() const;
    double as_number() const;
    const std::string& as_string() const;
    const Array& as_array() const;
    const Object& as_object() const;

    // The member with this key, or nullptr. Throws TypeError unless this is an object.
    const Value* find(const std::string& key) const;

private:
    Type type_ = Type::Null;
    bool bool_ = false;
    double number_ = 0;
    std::string string_;
    Array array_;
    Object object_;
};

// Strict RFC 8259 parsing. Duplicate keys and nesting beyond 200 levels are errors.
Value parse(const std::string& text);

// Compact when indent is 0, otherwise pretty-printed with that many spaces per level.
std::string dump(const Value& value, int indent = 0);

}  // namespace routeplan::json
