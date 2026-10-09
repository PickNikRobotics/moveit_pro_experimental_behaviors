# JSON Behaviors

These Behaviors let an Objective build, edit, read, send and receive JSON. Use them to talk to systems
that do not use ROS, such as fleet managers, dashboards or supervisors, without writing C++ for each
integration.

Source: `src/json_behaviors/` and `include/experimental_behaviors/json_behaviors/`.
Tests: `test/json_behaviors/`.

## How JSON is stored and addressed

- A JSON document is a `std::string` on the blackboard. Editing Behaviors write it back in compact form.
- A field is addressed with a JSON Pointer path (RFC 6901):

  | Path         | Meaning                                         |
  | ------------ | ----------------------------------------------- |
  | `""` (empty) | The whole document                              |
  | `/a/b`       | Key `b` inside key `a`                          |
  | `/items/0`   | First element of the array `items`              |
  | `/items/-`   | One past the last element (append, on set only) |
  | `/a~1b`      | Key `a/b` (`~1` stands for `/`)                 |
  | `/a~0b`      | Key `a~b` (`~0` stands for `~`)                 |

- JSON text can be typed straight into a port, for example `initial='{"robot": "arm"}'`. BT.CPP would
  normally read text in braces as a blackboard key; these Behaviors take it as JSON when it parses as
  JSON. (BT.CPP still adds an empty blackboard entry named after that text when it loads the tree.)

## Editing Behaviors

### CreateJson

Creates a document: an empty object, or a validated copy of a template.

| Data Port Name | Port Type | Object Type | Default | Description                     |
| -------------- | --------- | ----------- | ------- | ------------------------------- |
| initial        | input     | std::string | `{}`    | JSON text to start from         |
| json           | output    | std::string |         | The new document, compact JSON  |

### SetJsonField

Adds or replaces one field.

| Data Port Name      | Port Type | Object Type          | Default | Description                                         |
| ------------------- | --------- | -------------------- | ------- | --------------------------------------------------- |
| json                | inout     | std::string          |         | Document to edit                                    |
| path                | input     | std::string          |         | JSON Pointer of the field                           |
| value               | input     | any (AnyTypeAllowed) |         | Blackboard entry of any type, or a literal          |
| create_missing      | input     | bool                 | `true`  | Create missing parent objects                       |
| parse_value_as_json | input     | bool                 | `false` | Parse a string value as JSON instead of a string    |

- `std::string`, `bool` and number types map to the matching JSON type.
- Other types go through the JSON converters MoveIt Pro registers for the UI blackboard view (for
  example `geometry_msgs/msg/PoseStamped`), so the result matches what the UI shows, including the
  `__type` field. Those converters write numbers as strings with six significant digits.
- A literal typed into `value` is a JSON string unless `parse_value_as_json` is `true`.
- Missing parents are created as objects. An array index below the size replaces the element; an index
  equal to the size, or `-`, appends; a larger index fails.
- Fails on invalid JSON (with line and column), an invalid path, a parent that is a string, number or
  boolean, or a value whose C++ type has no JSON converter (the message names the type).

### GetJsonField

Reads one field.

| Data Port Name | Port Type | Object Type          | Default | Description                                   |
| -------------- | --------- | -------------------- | ------- | --------------------------------------------- |
| json           | input     | std::string          |         | Document to read                              |
| path           | input     | std::string          |         | JSON Pointer of the field                     |
| message_type   | input     | std::string          | `""`    | Optional ROS message type to convert to       |
| value          | output    | any (AnyTypeAllowed) |         | The field's value                             |

- Without `message_type`: a string, boolean or number keeps its type (`std::string`, `bool`, `int64_t`,
  `uint64_t` or `double`), so typed ports and Script expressions can use it. An object, array or null
  comes out as JSON text.
- With `message_type`: supported types are `geometry_msgs/msg/PoseStamped`, `Pose`, `Point`,
  `Quaternion` and `Vector3` (the `::` spelling also works). Numbers may be JSON numbers or numeric
  strings. In a PoseStamped, `header` is optional; all pose numbers are required.
- Fails if the path does not exist, or if the field does not match the requested type (the message
  names the field).

### RemoveJsonField

| Data Port Name | Port Type | Object Type | Description               |
| -------------- | --------- | ----------- | ------------------------- |
| json           | inout     | std::string | Document to edit          |
| path           | input     | std::string | JSON Pointer of the field |

Succeeds without a change if the field does not exist. Fails on the empty path.

### HasJsonField (condition)

| Data Port Name | Port Type | Object Type | Description               |
| -------------- | --------- | ----------- | ------------------------- |
| json           | input     | std::string | Document to check         |
| path           | input     | std::string | JSON Pointer of the field |

SUCCESS if the field exists (also when its value is null), FAILURE if not.

### MergeJson

Applies a JSON Merge Patch (RFC 7386).

| Data Port Name | Port Type | Object Type | Description         |
| -------------- | --------- | ----------- | ------------------- |
| json           | inout     | std::string | Document to edit    |
| patch          | input     | std::string | Merge patch to apply |

Objects merge key by key, a `null` removes the key, and any other value (arrays too) replaces it.

## Transport Behaviors

### SendJsonUdp

Sends a document as one UDP datagram.

| Data Port Name | Port Type | Object Type | Default     | Description                     |
| -------------- | --------- | ----------- | ----------- | ------------------------------- |
| host           | input     | std::string | `127.0.0.1` | Destination address or host name |
| port           | input     | int         |             | Destination UDP port            |
| payload        | input     | std::string | `{json}`    | JSON document to send           |

UDP does not confirm delivery. The payload must be valid JSON and at most 65507 bytes.

### ReceiveJsonUdp

Waits for a document sent as a UDP datagram.

| Data Port Name | Port Type | Object Type | Default     | Description                                 |
| -------------- | --------- | ----------- | ----------- | ------------------------------------------- |
| port           | input     | int         |             | UDP port to listen on                       |
| bind_address   | input     | std::string | `127.0.0.1` | Local address to listen on                  |
| timeout        | input     | double      | `1.0`       | Seconds to wait; `0` checks once            |
| payload        | output    | std::string | `{payload}` | The newest datagram, as JSON text           |

- The socket opens on the first tick and stays open until the tree is destroyed, so datagrams that
  arrive between runs are kept. Datagrams sent before the first tick are not.
- Each run outputs the newest queued datagram and drops older ones. With nothing queued it returns
  RUNNING, and FAILURE after `timeout`. A datagram that is not valid JSON is a FAILURE.

### Sending JSON over HTTP

`SendJsonHttp` was replaced by the generic `SendHttpRequest` Behavior; see
[restful_behaviors.md](restful_behaviors.md), which also has the port-by-port mapping. To send a JSON
document, put it in `body` (the default `content_type` is `application/json`).

## Security

- **UDP is not an authenticated channel.** Any process that can reach the port can send a datagram,
  and the sender address is not checked.
- **Bind the receiver to a chosen interface.** `bind_address` defaults to `127.0.0.1`, which accepts
  datagrams from this computer only. Set the address of one network interface to accept datagrams from
  that network. Use `0.0.0.0` (every interface) only on a trusted network.
- **Treat received JSON as untrusted input.** Check every field you read (with `HasJsonField` and range
  checks in the tree) before it drives motion.
- **HTTP sends in clear text.** Use `https://` with `SendHttpRequest` when the request crosses a
  network you do not control.

## Example

Report a pose to a supervisor, then wait for its reply on UDP:

```xml
<Sequence>
  <CreateJson initial='{"robot": "arm_1", "status": {}}' json="{msg}"/>
  <SetJsonField json="{msg}" path="/status/state" value="picking"/>
  <SetJsonField json="{msg}" path="/status/pose" value="{tool_pose}"/>
  <SendHttpRequest url="http://192.168.1.20:8080/status" method="POST" body="{msg}" response_body="{reply}"/>
  <ReceiveJsonUdp port="9870" bind_address="192.168.1.5" timeout="5.0" payload="{command}"/>
  <HasJsonField json="{command}" path="/target"/>
  <GetJsonField json="{command}" path="/target" message_type="geometry_msgs/msg/PoseStamped"
                value="{target_pose}"/>
</Sequence>
```
