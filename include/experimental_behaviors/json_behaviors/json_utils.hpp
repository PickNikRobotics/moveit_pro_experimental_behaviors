// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/utils/safe_any.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

/**
 * @brief Helpers shared by the JSON Behaviors.
 *
 * @details
 * A JSON document travels on the blackboard as a std::string. Fields are addressed with JSON Pointer
 * paths (RFC 6901): "" is the whole document, "/a/b" is a nested key, "/items/0" is an array element,
 * "~1" escapes "/" and "~0" escapes "~" inside a key.
 *
 * Every function returns an error string instead of throwing, so a Behavior can report it and fail.
 */
namespace experimental_behaviors::json_utils
{
using Json = nlohmann::json;

/// Parses JSON text. The error names @p what and gives the parse location (line and column).
tl::expected<Json, std::string> parse(const std::string& text, const std::string& what);

/**
 * @brief Reads a string input port that holds JSON text.
 * @details BT.CPP reads any port text wrapped in braces as a blackboard key, which would swallow a JSON
 * object typed into the port. Text that parses as JSON is taken as a literal; a `{key}` reference never does.
 */
tl::expected<std::string, std::string> getJsonText(const BT::TreeNode& node, const std::string& port);

/// Returns true if @p port holds literal JSON text that BT.CPP would otherwise read as a blackboard key.
bool isJsonLiteralPort(const BT::TreeNode& node, const std::string& port);

/// Parses an RFC 6901 JSON Pointer into its unescaped reference tokens.
tl::expected<std::vector<std::string>, std::string> parsePointer(const std::string& path);

/// Renders reference tokens back into an escaped JSON Pointer, for messages.
std::string pointerToString(const std::vector<std::string>& tokens, std::size_t count);

/// Returns the JSON type name of @p value ("object", "array", "string", "number", "boolean", "null").
std::string typeName(const Json& value);

/// Finds the element at @p tokens. The error says why the path does not resolve.
tl::expected<const Json*, std::string> find(const Json& doc, const std::vector<std::string>& tokens);

/**
 * @brief Adds or replaces the element at @p tokens.
 * @param create_missing If true, missing parents (and null parents) are created as objects.
 * @details An array index below the size replaces that element. An index equal to the size, or "-",
 * appends. A larger index, or descending into a string, number or boolean, is an error.
 */
tl::expected<void, std::string> set(Json& doc, const std::vector<std::string>& tokens, Json value, bool create_missing);

/// Removes the element at @p tokens. Returns false (and changes nothing) if it does not exist.
tl::expected<bool, std::string> remove(Json& doc, const std::vector<std::string>& tokens);

/**
 * @brief Converts a blackboard value to JSON.
 * @details Strings, booleans and numbers map directly. Any other type goes through the converters
 * registered with BT::JsonExporter, the same path the UI blackboard view uses.
 */
tl::expected<Json, std::string> anyToJson(const BT::Any& value);

/// Converts a JSON element to a blackboard value: scalars keep their type, other values become JSON text.
BT::Any jsonToAny(const Json& value);

/**
 * @brief Converts a JSON element to the ROS message named by @p message_type (see supportedMessageTypes()).
 * @param path JSON Pointer of @p value in its document, used to name the failing field in errors.
 */
tl::expected<BT::Any, std::string> jsonToMessage(const Json& value, const std::string& message_type,
                                                 const std::string& path);

/// Lists the message types jsonToMessage() accepts, for port descriptions and errors.
std::string supportedMessageTypes();
}  // namespace experimental_behaviors::json_utils
