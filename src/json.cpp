#include "routeplan/json.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>

namespace routeplan::json {

ParseError::ParseError(const std::string& message, int line, int column)
    : std::runtime_error("line " + std::to_string(line) + ", column " + std::to_string(column) + ": " + message),
      line_(line),
      column_(column) {}

bool Value::as_bool() const {
    if (type_ != Type::Bool) throw TypeError("expected true or false");
    return bool_;
}

double Value::as_number() const {
    if (type_ != Type::Number) throw TypeError("expected a number");
    return number_;
}

const std::string& Value::as_string() const {
    if (type_ != Type::String) throw TypeError("expected a string");
    return string_;
}

const Value::Array& Value::as_array() const {
    if (type_ != Type::Array) throw TypeError("expected an array");
    return array_;
}

const Value::Object& Value::as_object() const {
    if (type_ != Type::Object) throw TypeError("expected an object");
    return object_;
}

const Value* Value::find(const std::string& key) const {
    for (const Member& member : as_object()) {
        if (member.first == key) return &member.second;
    }
    return nullptr;
}

namespace {

constexpr int kMaxDepth = 200;

class Parser {
public:
    explicit Parser(const std::string& text) : text_(text) {}

    Value document() {
        skip_space();
        Value value = parse_value(0);
        skip_space();
        if (pos_ != text_.size()) fail("unexpected text after the value");
        return value;
    }

private:
    [[noreturn]] void fail(const std::string& message) const {
        int line = 1;
        int column = 1;
        for (std::size_t i = 0; i < pos_ && i < text_.size(); ++i) {
            if (text_[i] == '\n') {
                ++line;
                column = 1;
            } else {
                ++column;
            }
        }
        throw ParseError(message, line, column);
    }

    bool at_end() const { return pos_ >= text_.size(); }
    char peek() const { return text_[pos_]; }

    void skip_space() {
        while (!at_end() && (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r')) ++pos_;
    }

    Value parse_value(int depth) {
        if (depth > kMaxDepth) fail("nesting is too deep");
        if (at_end()) fail("unexpected end of input");
        switch (peek()) {
            case '{': return parse_object(depth);
            case '[': return parse_array(depth);
            case '"': return Value(parse_string());
            case 't': expect_word("true"); return Value(true);
            case 'f': expect_word("false"); return Value(false);
            case 'n': expect_word("null"); return Value(nullptr);
            default: return parse_number();
        }
    }

    void expect_word(const std::string& word) {
        if (text_.compare(pos_, word.size(), word) != 0) fail("unexpected character");
        pos_ += word.size();
    }

    Value parse_object(int depth) {
        ++pos_;
        Value::Object members;
        skip_space();
        if (!at_end() && peek() == '}') {
            ++pos_;
            return Value(std::move(members));
        }
        while (true) {
            skip_space();
            if (at_end() || peek() != '"') fail("expected a string key");
            std::size_t key_start = pos_;
            std::string key = parse_string();
            for (const Value::Member& member : members) {
                if (member.first == key) {
                    pos_ = key_start;
                    fail("duplicate key \"" + key + "\"");
                }
            }
            skip_space();
            if (at_end() || peek() != ':') fail("expected ':'");
            ++pos_;
            skip_space();
            members.emplace_back(std::move(key), parse_value(depth + 1));
            skip_space();
            if (at_end()) fail("unexpected end of input");
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            if (peek() == '}') {
                ++pos_;
                return Value(std::move(members));
            }
            fail("expected ',' or '}'");
        }
    }

    Value parse_array(int depth) {
        ++pos_;
        Value::Array items;
        skip_space();
        if (!at_end() && peek() == ']') {
            ++pos_;
            return Value(std::move(items));
        }
        while (true) {
            skip_space();
            items.push_back(parse_value(depth + 1));
            skip_space();
            if (at_end()) fail("unexpected end of input");
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            if (peek() == ']') {
                ++pos_;
                return Value(std::move(items));
            }
            fail("expected ',' or ']'");
        }
    }

    unsigned parse_hex4() {
        unsigned code = 0;
        for (int i = 0; i < 4; ++i) {
            if (at_end()) fail("unfinished \\u escape");
            char c = peek();
            unsigned digit;
            if (c >= '0' && c <= '9') digit = static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') digit = static_cast<unsigned>(c - 'a') + 10;
            else if (c >= 'A' && c <= 'F') digit = static_cast<unsigned>(c - 'A') + 10;
            else fail("bad \\u escape");
            code = code * 16 + digit;
            ++pos_;
        }
        return code;
    }

    static void append_utf8(std::string& out, unsigned code) {
        if (code < 0x80) {
            out += static_cast<char>(code);
        } else if (code < 0x800) {
            out += static_cast<char>(0xC0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else if (code < 0x10000) {
            out += static_cast<char>(0xE0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (code >> 18));
            out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
    }

    std::string parse_string() {
        ++pos_;
        std::string out;
        while (true) {
            if (at_end()) fail("unterminated string");
            char c = text_[pos_++];
            if (c == '"') return out;
            if (static_cast<unsigned char>(c) < 0x20) {
                --pos_;
                fail("control character in string");
            }
            if (c != '\\') {
                out += c;
                continue;
            }
            if (at_end()) fail("unterminated string");
            char escape = text_[pos_++];
            switch (escape) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned code = parse_hex4();
                    if (code >= 0xD800 && code <= 0xDBFF) {
                        if (text_.compare(pos_, 2, "\\u") != 0) fail("lone surrogate in \\u escape");
                        pos_ += 2;
                        unsigned low = parse_hex4();
                        if (low < 0xDC00 || low > 0xDFFF) fail("lone surrogate in \\u escape");
                        code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
                    } else if (code >= 0xDC00 && code <= 0xDFFF) {
                        fail("lone surrogate in \\u escape");
                    }
                    append_utf8(out, code);
                    break;
                }
                default:
                    --pos_;
                    fail("bad escape");
            }
        }
    }

    static bool is_digit(char c) { return c >= '0' && c <= '9'; }

    Value parse_number() {
        std::size_t start = pos_;
        if (!at_end() && peek() == '-') ++pos_;
        if (at_end() || !is_digit(peek())) {
            pos_ = start;
            fail("unexpected character");
        }
        if (peek() == '0') {
            ++pos_;
        } else {
            while (!at_end() && is_digit(peek())) ++pos_;
        }
        if (!at_end() && peek() == '.') {
            ++pos_;
            if (at_end() || !is_digit(peek())) fail("expected a digit after '.'");
            while (!at_end() && is_digit(peek())) ++pos_;
        }
        if (!at_end() && (peek() == 'e' || peek() == 'E')) {
            ++pos_;
            if (!at_end() && (peek() == '+' || peek() == '-')) ++pos_;
            if (at_end() || !is_digit(peek())) fail("expected a digit in the exponent");
            while (!at_end() && is_digit(peek())) ++pos_;
        }
        if (!at_end() && is_digit(peek())) fail("number has a leading zero");
        double value = 0;
        auto result = std::from_chars(text_.data() + start, text_.data() + pos_, value);
        if (result.ec != std::errc()) {
            pos_ = start;
            fail("number is out of range");
        }
        return Value(value);
    }

    const std::string& text_;
    std::size_t pos_ = 0;
};

void dump_string(const std::string& text, std::string& out) {
    out += '"';
    for (char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof buffer, "\\u%04x", static_cast<unsigned>(c));
                    out += buffer;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void dump_number(double value, std::string& out) {
    if (!std::isfinite(value)) {
        out += "null";
        return;
    }
    char buffer[64];
    std::to_chars_result result{};
    if (value == std::floor(value) && std::fabs(value) < 1e15) {
        result = std::to_chars(buffer, buffer + sizeof buffer, static_cast<long long>(value));
    } else {
        result = std::to_chars(buffer, buffer + sizeof buffer, value);
    }
    out.append(buffer, result.ptr);
}

void newline(std::string& out, int indent, int level) {
    if (indent > 0) {
        out += '\n';
        out.append(static_cast<std::size_t>(indent * level), ' ');
    }
}

void dump_value(const Value& value, int indent, int level, std::string& out) {
    switch (value.type()) {
        case Value::Type::Null: out += "null"; break;
        case Value::Type::Bool: out += value.as_bool() ? "true" : "false"; break;
        case Value::Type::Number: dump_number(value.as_number(), out); break;
        case Value::Type::String: dump_string(value.as_string(), out); break;
        case Value::Type::Array: {
            const Value::Array& items = value.as_array();
            if (items.empty()) {
                out += "[]";
                break;
            }
            out += '[';
            for (std::size_t i = 0; i < items.size(); ++i) {
                if (i > 0) out += ',';
                newline(out, indent, level + 1);
                dump_value(items[i], indent, level + 1, out);
            }
            newline(out, indent, level);
            out += ']';
            break;
        }
        case Value::Type::Object: {
            const Value::Object& members = value.as_object();
            if (members.empty()) {
                out += "{}";
                break;
            }
            out += '{';
            for (std::size_t i = 0; i < members.size(); ++i) {
                if (i > 0) out += ',';
                newline(out, indent, level + 1);
                dump_string(members[i].first, out);
                out += indent > 0 ? ": " : ":";
                dump_value(members[i].second, indent, level + 1, out);
            }
            newline(out, indent, level);
            out += '}';
            break;
        }
    }
}

}  // namespace

Value parse(const std::string& text) {
    return Parser(text).document();
}

std::string dump(const Value& value, int indent) {
    std::string out;
    dump_value(value, indent, 0, out);
    return out;
}

}  // namespace routeplan::json
