// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/json_utils.hpp>

#include <behaviortree_cpp/json_export.h>

#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <system_error>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

namespace experimental_behaviors::json_utils
{
namespace
{
/// Parses an array index token: "0" or a number with no leading zero. "-" is handled by the callers.
std::optional<std::size_t> parseIndex(const std::string& token)
{
  if (token.empty() || (token.size() > 1 && token.front() == '0'))
  {
    return std::nullopt;
  }
  std::size_t index = 0;
  const auto* end = token.data() + token.size();
  const auto [ptr, ec] = std::from_chars(token.data(), end, index);
  if (ec != std::errc() || ptr != end)
  {
    return std::nullopt;
  }
  return index;
}

std::string invalidIndexMessage(const std::string& token, const std::vector<std::string>& tokens, std::size_t depth)
{
  return "'" + token + "' is not a valid array index for the array at '" + pointerToString(tokens, depth) +
         "' (use a number without leading zeros, or '-' to append)";
}

// The message readers below take the JSON Pointer of the element they read, so every error names it.
tl::expected<const Json*, std::string> member(const Json& object, const std::string& key, const std::string& where)
{
  if (!object.is_object())
  {
    return tl::make_unexpected("'" + where + "' is " + typeName(object) + ", expected an object");
  }
  const auto it = object.find(key);
  if (it == object.end())
  {
    return tl::make_unexpected("missing field '" + where + "/" + key + "'");
  }
  return &(*it);
}

tl::expected<double, std::string> readDouble(const Json& object, const std::string& key, const std::string& where)
{
  const auto field = member(object, key, where);
  if (!field)
  {
    return tl::make_unexpected(field.error());
  }
  const Json& value = **field;
  if (value.is_number())
  {
    return value.get<double>();
  }
  if (value.is_string())
  {
    // Pro's blackboard converter renders numbers as strings, so accept a fully numeric string.
    const auto& text = value.get_ref<const std::string&>();
    double parsed = 0.0;
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(text.data(), end, parsed);
    if (ec == std::errc() && ptr == end && std::isfinite(parsed))
    {
      return parsed;
    }
    return tl::make_unexpected("'" + where + "/" + key + "' is the string \"" + text + "\", expected a number");
  }
  return tl::make_unexpected("'" + where + "/" + key + "' is " + typeName(value) + ", expected a number");
}

template <typename IntT>
tl::expected<IntT, std::string> readInteger(const Json& object, const std::string& key, const std::string& where)
{
  const auto field = member(object, key, where);
  if (!field)
  {
    return tl::make_unexpected(field.error());
  }
  const Json& value = **field;
  const std::string label = "'" + where + "/" + key + "'";
  int64_t parsed = 0;
  if (value.is_number_integer())
  {
    parsed = value.get<int64_t>();
  }
  else if (value.is_string())
  {
    const auto& text = value.get_ref<const std::string&>();
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(text.data(), end, parsed);
    if (ec != std::errc() || ptr != end)
    {
      return tl::make_unexpected(label + " is the string \"" + text + "\", expected an integer");
    }
  }
  else
  {
    return tl::make_unexpected(label + " is " + typeName(value) + ", expected an integer");
  }
  if (parsed < static_cast<int64_t>(std::numeric_limits<IntT>::min()) ||
      parsed > static_cast<int64_t>(std::numeric_limits<IntT>::max()))
  {
    return tl::make_unexpected(label + " is out of range");
  }
  return static_cast<IntT>(parsed);
}

// Each reader fills one message from the object at `where`; the first missing or mistyped field fails it.
tl::expected<geometry_msgs::msg::Point, std::string> readPoint(const Json& json, const std::string& where)
{
  geometry_msgs::msg::Point point;
  for (auto [key, target] : { std::pair{ "x", &point.x }, std::pair{ "y", &point.y }, std::pair{ "z", &point.z } })
  {
    const auto value = readDouble(json, key, where);
    if (!value)
    {
      return tl::make_unexpected(value.error());
    }
    *target = *value;
  }
  return point;
}

tl::expected<geometry_msgs::msg::Vector3, std::string> readVector3(const Json& json, const std::string& where)
{
  const auto point = readPoint(json, where);
  if (!point)
  {
    return tl::make_unexpected(point.error());
  }
  geometry_msgs::msg::Vector3 vector;
  vector.x = point->x;
  vector.y = point->y;
  vector.z = point->z;
  return vector;
}

tl::expected<geometry_msgs::msg::Quaternion, std::string> readQuaternion(const Json& json, const std::string& where)
{
  geometry_msgs::msg::Quaternion quaternion;
  for (auto [key, target] : { std::pair{ "x", &quaternion.x }, std::pair{ "y", &quaternion.y },
                              std::pair{ "z", &quaternion.z }, std::pair{ "w", &quaternion.w } })
  {
    const auto value = readDouble(json, key, where);
    if (!value)
    {
      return tl::make_unexpected(value.error());
    }
    *target = *value;
  }
  return quaternion;
}

tl::expected<geometry_msgs::msg::Pose, std::string> readPose(const Json& json, const std::string& where)
{
  geometry_msgs::msg::Pose pose;
  const auto position = member(json, "position", where);
  if (!position)
  {
    return tl::make_unexpected(position.error());
  }
  const auto point = readPoint(**position, where + "/position");
  if (!point)
  {
    return tl::make_unexpected(point.error());
  }
  const auto orientation = member(json, "orientation", where);
  if (!orientation)
  {
    return tl::make_unexpected(orientation.error());
  }
  const auto quaternion = readQuaternion(**orientation, where + "/orientation");
  if (!quaternion)
  {
    return tl::make_unexpected(quaternion.error());
  }
  pose.position = *point;
  pose.orientation = *quaternion;
  return pose;
}

// The header is optional: external systems often omit it, and an empty frame_id is the message default.
tl::expected<std_msgs::msg::Header, std::string> readHeader(const Json& json, const std::string& where)
{
  std_msgs::msg::Header header;
  if (!json.contains("header"))
  {
    return header;
  }
  const Json& header_json = json.at("header");
  const std::string header_where = where + "/header";
  if (!header_json.is_object())
  {
    return tl::make_unexpected("'" + header_where + "' is " + typeName(header_json) + ", expected an object");
  }
  if (header_json.contains("frame_id"))
  {
    const Json& frame_id = header_json.at("frame_id");
    if (!frame_id.is_string())
    {
      return tl::make_unexpected("'" + header_where + "/frame_id' is " + typeName(frame_id) + ", expected a string");
    }
    header.frame_id = frame_id.get<std::string>();
  }
  if (header_json.contains("stamp"))
  {
    const Json& stamp = header_json.at("stamp");
    const std::string stamp_where = header_where + "/stamp";
    const auto sec = readInteger<int32_t>(stamp, "sec", stamp_where);
    if (!sec)
    {
      return tl::make_unexpected(sec.error());
    }
    const auto nanosec = readInteger<uint32_t>(stamp, "nanosec", stamp_where);
    if (!nanosec)
    {
      return tl::make_unexpected(nanosec.error());
    }
    header.stamp.sec = *sec;
    header.stamp.nanosec = *nanosec;
  }
  return header;
}

tl::expected<geometry_msgs::msg::PoseStamped, std::string> readPoseStamped(const Json& json, const std::string& where)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  const auto header = readHeader(json, where);
  if (!header)
  {
    return tl::make_unexpected(header.error());
  }
  const auto pose_json = member(json, "pose", where);
  if (!pose_json)
  {
    return tl::make_unexpected(pose_json.error());
  }
  const auto pose = readPose(**pose_json, where + "/pose");
  if (!pose)
  {
    return tl::make_unexpected(pose.error());
  }
  pose_stamped.header = *header;
  pose_stamped.pose = *pose;
  return pose_stamped;
}

template <typename MessageT>
tl::expected<BT::Any, std::string> toAny(const tl::expected<MessageT, std::string>& message)
{
  if (!message)
  {
    return tl::make_unexpected(message.error());
  }
  return BT::Any(*message);
}

/// Normalizes "pkg::msg::Type" (the UI's __type spelling) to "pkg/msg/Type".
std::string normalizeTypeName(std::string type)
{
  for (auto pos = type.find("::"); pos != std::string::npos; pos = type.find("::", pos + 1))
  {
    type.replace(pos, 2, "/");
  }
  return type;
}
}  // namespace

tl::expected<Json, std::string> parse(const std::string& text, const std::string& what)
{
  try
  {
    return Json::parse(text);
  }
  catch (const Json::parse_error& e)
  {
    // nlohmann's message already carries "parse error at line L, column C".
    return tl::make_unexpected(what + " is not valid JSON: " + e.what());
  }
}

bool isJsonLiteralPort(const BT::TreeNode& node, const std::string& port)
{
  const auto& ports = node.config().input_ports;
  const auto it = ports.find(port);
  return it != ports.end() && BT::TreeNode::isBlackboardPointer(it->second) && Json::accept(it->second);
}

tl::expected<std::string, std::string> getJsonText(const BT::TreeNode& node, const std::string& port)
{
  if (isJsonLiteralPort(node, port))
  {
    return node.config().input_ports.at(port);
  }
  auto text = node.getInput<std::string>(port);
  if (!text)
  {
    return tl::make_unexpected("Failed to get input [" + port + "]: " + text.error());
  }
  return text.value();
}

tl::expected<std::vector<std::string>, std::string> parsePointer(const std::string& path)
{
  Json::json_pointer pointer;
  try
  {
    pointer = Json::json_pointer(path);
  }
  catch (const Json::exception& e)
  {
    return tl::make_unexpected("'" + path + "' is not a valid JSON Pointer (RFC 6901): " + e.what());
  }
  std::vector<std::string> tokens;
  while (!pointer.empty())
  {
    tokens.insert(tokens.begin(), pointer.back());
    pointer.pop_back();
  }
  return tokens;
}

std::string pointerToString(const std::vector<std::string>& tokens, std::size_t count)
{
  std::string path;
  for (std::size_t i = 0; i < count && i < tokens.size(); ++i)
  {
    path += '/';
    for (const char c : tokens[i])
    {
      if (c == '~')
      {
        path += "~0";
      }
      else if (c == '/')
      {
        path += "~1";
      }
      else
      {
        path += c;
      }
    }
  }
  return path;
}

std::string typeName(const Json& value)
{
  switch (value.type())
  {
    case Json::value_t::object:
      return "an object";
    case Json::value_t::array:
      return "an array";
    case Json::value_t::string:
      return "a string";
    case Json::value_t::boolean:
      return "a boolean";
    case Json::value_t::number_integer:
    case Json::value_t::number_unsigned:
    case Json::value_t::number_float:
      return "a number";
    case Json::value_t::null:
      return "null";
    default:
      return "an unsupported value";
  }
}

tl::expected<const Json*, std::string> find(const Json& doc, const std::vector<std::string>& tokens)
{
  const Json* current = &doc;
  for (std::size_t depth = 0; depth < tokens.size(); ++depth)
  {
    const std::string& token = tokens[depth];
    const std::string missing = "no field at '" + pointerToString(tokens, tokens.size()) + "': ";
    if (current->is_object())
    {
      const auto it = current->find(token);
      if (it == current->end())
      {
        return tl::make_unexpected(missing + "the object at '" + pointerToString(tokens, depth) + "' has no key '" +
                                   token + "'");
      }
      current = &(*it);
    }
    else if (current->is_array())
    {
      const auto index = parseIndex(token);
      if (!index || *index >= current->size())
      {
        return tl::make_unexpected(missing + "the array at '" + pointerToString(tokens, depth) + "' has " +
                                   std::to_string(current->size()) + " elements and no element '" + token + "'");
      }
      current = &(*current)[*index];
    }
    else
    {
      return tl::make_unexpected(missing + "'" + pointerToString(tokens, depth) + "' is " + typeName(*current) +
                                 ", which has no fields");
    }
  }
  return current;
}

tl::expected<void, std::string> set(Json& doc, const std::vector<std::string>& tokens, Json value, bool create_missing)
{
  if (tokens.empty())
  {
    doc = std::move(value);
    return {};
  }
  Json* current = &doc;
  for (std::size_t depth = 0; depth < tokens.size(); ++depth)
  {
    const std::string& token = tokens[depth];
    const bool is_leaf = depth + 1 == tokens.size();
    if (current->is_null() && create_missing)
    {
      *current = Json::object();
    }
    if (current->is_object())
    {
      if (is_leaf)
      {
        (*current)[token] = std::move(value);
        return {};
      }
      auto it = current->find(token);
      if (it == current->end())
      {
        if (!create_missing)
        {
          return tl::make_unexpected("parent '" + pointerToString(tokens, depth + 1) +
                                     "' does not exist and create_missing is false");
        }
        it = current->emplace(token, Json::object()).first;
      }
      current = &(*it);
    }
    else if (current->is_array())
    {
      std::size_t index = current->size();
      if (token != "-")
      {
        const auto parsed = parseIndex(token);
        if (!parsed)
        {
          return tl::make_unexpected(invalidIndexMessage(token, tokens, depth));
        }
        index = *parsed;
      }
      if (index > current->size())
      {
        return tl::make_unexpected("index " + token + " is past the end of the array at '" +
                                   pointerToString(tokens, depth) + "', which has " + std::to_string(current->size()) +
                                   " elements");
      }
      if (index == current->size())
      {
        if (!is_leaf && !create_missing)
        {
          return tl::make_unexpected("parent '" + pointerToString(tokens, depth + 1) +
                                     "' does not exist and create_missing is false");
        }
        current->push_back(is_leaf ? std::move(value) : Json::object());
        if (is_leaf)
        {
          return {};
        }
        current = &current->back();
      }
      else if (is_leaf)
      {
        (*current)[index] = std::move(value);
        return {};
      }
      else
      {
        current = &(*current)[index];
      }
    }
    else
    {
      return tl::make_unexpected("cannot set '" + pointerToString(tokens, tokens.size()) + "': '" +
                                 pointerToString(tokens, depth) + "' is " + typeName(*current) +
                                 ", which cannot hold fields");
    }
  }
  return {};
}

tl::expected<bool, std::string> remove(Json& doc, const std::vector<std::string>& tokens)
{
  if (tokens.empty())
  {
    return tl::make_unexpected("cannot remove the document root (path \"\")");
  }
  const std::vector<std::string> parent_tokens(tokens.begin(), tokens.end() - 1);
  const auto parent = find(doc, parent_tokens);
  if (!parent)
  {
    return false;
  }
  // find() hands back a const view into doc; doc itself is mutable, so dropping const here is safe.
  Json& container = const_cast<Json&>(**parent);
  const std::string& token = tokens.back();
  if (container.is_object())
  {
    return container.erase(token) > 0;
  }
  if (container.is_array())
  {
    const auto index = parseIndex(token);
    if (!index || *index >= container.size())
    {
      return false;
    }
    container.erase(*index);
    return true;
  }
  return false;
}

tl::expected<Json, std::string> anyToJson(const BT::Any& value)
{
  if (value.empty())
  {
    return tl::make_unexpected("the value is empty");
  }
  if (value.isType<bool>())
  {
    return Json(value.cast<bool>());
  }
  if (value.isString())
  {
    return Json(value.cast<std::string>());
  }
  if (value.castedType() == typeid(double))
  {
    const double number = value.cast<double>();
    if (!std::isfinite(number))
    {
      return tl::make_unexpected("the value " + std::to_string(number) + " is not a finite number");
    }
    return Json(number);
  }
  if (value.castedType() == typeid(int64_t))
  {
    return Json(value.cast<int64_t>());
  }
  if (value.castedType() == typeid(uint64_t))
  {
    return Json(value.cast<uint64_t>());
  }
  Json json;
  bool converted = false;
  try
  {
    converted = BT::JsonExporter::get().toJson(value, json);
  }
  catch (const std::exception& e)
  {
    return tl::make_unexpected("the JSON converter for C++ type '" + BT::demangle(value.type()) +
                               "' failed: " + e.what());
  }
  if (!converted)
  {
    return tl::make_unexpected("cannot serialize a value of C++ type '" + BT::demangle(value.type()) +
                               "': no JSON converter is registered for it");
  }
  return json;
}

BT::Any jsonToAny(const Json& value)
{
  switch (value.type())
  {
    case Json::value_t::string:
      return BT::Any(value.get<std::string>());
    case Json::value_t::boolean:
      return BT::Any(value.get<bool>());
    case Json::value_t::number_integer:
      return BT::Any(value.get<int64_t>());
    case Json::value_t::number_unsigned:
      return BT::Any(value.get<uint64_t>());
    case Json::value_t::number_float:
      return BT::Any(value.get<double>());
    default:
      return BT::Any(value.dump());
  }
}

tl::expected<BT::Any, std::string> jsonToMessage(const Json& value, const std::string& message_type,
                                                 const std::string& path)
{
  const std::string type = normalizeTypeName(message_type);
  if (value.is_object() && value.contains("__type"))
  {
    const Json& tag = value.at("__type");
    if (tag.is_string() && normalizeTypeName(tag.get<std::string>()) != type)
    {
      return tl::make_unexpected("'" + path + "' is tagged as '" + tag.get<std::string>() + "', not '" + message_type +
                                 "'");
    }
  }
  const std::string& where = path;
  if (type == "geometry_msgs/msg/PoseStamped")
  {
    return toAny(readPoseStamped(value, where));
  }
  if (type == "geometry_msgs/msg/Pose")
  {
    return toAny(readPose(value, where));
  }
  if (type == "geometry_msgs/msg/Point")
  {
    return toAny(readPoint(value, where));
  }
  if (type == "geometry_msgs/msg/Quaternion")
  {
    return toAny(readQuaternion(value, where));
  }
  if (type == "geometry_msgs/msg/Vector3")
  {
    return toAny(readVector3(value, where));
  }
  return tl::make_unexpected("unsupported message_type '" + message_type + "' (supported: " + supportedMessageTypes() +
                             ")");
}

std::string supportedMessageTypes()
{
  return "geometry_msgs/msg/PoseStamped, geometry_msgs/msg/Pose, geometry_msgs/msg/Point, "
         "geometry_msgs/msg/Quaternion, geometry_msgs/msg/Vector3";
}
}  // namespace experimental_behaviors::json_utils
