#pragma once
// Test-only reader for the shared v1 byte vectors (docs/protocol/fixtures/v1-frames.txt,
// #161). The desktop tests read the same file, so both endpoints are compared against
// one immutable set of bytes. Allocation is fine here: this is host test code.
#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef CODECELL_PROTOCOL_V1_VECTORS
#error "CODECELL_PROTOCOL_V1_VECTORS must name docs/protocol/fixtures/v1-frames.txt"
#endif

namespace codecell::protocol::fixtures {

struct Vector {
    std::string name;
    std::string kind;
    std::string source;
    std::map<std::string, std::string> fields;
    std::vector<std::uint8_t> bytes;

    [[nodiscard]] const std::string& field(const std::string& key) const
    {
        const auto it = fields.find(key);
        if (it == fields.end()) throw std::runtime_error(name + ": missing field " + key);
        return it->second;
    }
    [[nodiscard]] bool isNull(const std::string& key) const { return field(key) == "null"; }
    [[nodiscard]] long long integer(const std::string& key) const { return std::stoll(field(key)); }
    [[nodiscard]] std::optional<long long> optionalInteger(const std::string& key) const
    {
        if (isNull(key)) return std::nullopt;
        return integer(key);
    }
    [[nodiscard]] bool boolean(const std::string& key) const
    {
        const auto& value = field(key);
        if (value != "true" && value != "false") throw std::runtime_error(name + ": not a boolean: " + key);
        return value == "true";
    }
};

[[nodiscard]] inline std::vector<Vector> loadVectors()
{
    std::ifstream file(CODECELL_PROTOCOL_V1_VECTORS);
    if (!file) throw std::runtime_error("cannot open " CODECELL_PROTOCOL_V1_VECTORS);
    std::vector<Vector> vectors;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line.front() == '#') continue;
        const auto bar = line.find('|');
        if (bar == std::string::npos) throw std::runtime_error("vector without bytes: " + line);
        Vector vector;
        std::istringstream head(line.substr(0, bar));
        head >> vector.name >> vector.kind >> vector.source;
        for (std::string token; head >> token;) {
            const auto equals = token.find('=');
            if (equals == std::string::npos) throw std::runtime_error("bad field: " + token);
            vector.fields[token.substr(0, equals)] = token.substr(equals + 1);
        }
        std::istringstream hex(line.substr(bar + 1));
        for (std::string byte; hex >> byte;) vector.bytes.push_back(static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16)));
        vectors.push_back(std::move(vector));
    }
    return vectors;
}

[[nodiscard]] inline std::vector<Vector> vectorsOfKind(const std::string& kind)
{
    std::vector<Vector> result;
    for (auto& vector : loadVectors())
        if (vector.kind == kind) result.push_back(std::move(vector));
    return result;
}

[[nodiscard]] inline Vector vectorNamed(const std::string& name)
{
    for (auto& vector : loadVectors())
        if (vector.name == name) return vector;
    throw std::runtime_error("no vector named " + name);
}
} // namespace codecell::protocol::fixtures
