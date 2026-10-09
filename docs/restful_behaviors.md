# REST Behaviors

`SendHttpRequest` lets an Objective call a REST API: it sends one HTTP request (GET, POST, PUT, PATCH
or DELETE) and puts the response on the blackboard. Use it to read from or write to systems that do not
use ROS, such as fleet managers, PLC gateways, dashboards or job servers.

Source: `src/restful_behaviors/` and `include/experimental_behaviors/restful_behaviors/`.
Tests: `test/restful_behaviors/`.
To copy the Behavior into your own workspace, follow
[restful_behaviors_copy_howto.html](restful_behaviors_copy_howto.html) (one standalone page).

## SendHttpRequest

| Data Port Name   | Port Type | Object Type | Default              | Description                                               |
| ---------------- | --------- | ----------- | -------------------- | --------------------------------------------------------- |
| url              | input     | std::string |                      | `http://` or `https://` URL                               |
| method           | input     | std::string | `GET`                | `GET`, `POST`, `PUT`, `PATCH` or `DELETE` (any case)      |
| query_parameters | input     | std::string | `""`                 | JSON object of query parameters, as text                  |
| headers          | input     | std::string | `""`                 | JSON object of request headers, as text                   |
| body             | input     | std::string | `""`                 | Request body, sent as it is                               |
| content_type     | input     | std::string | `application/json`   | Content-Type of the body; empty sends none                |
| timeout          | input     | double      | `10.0`               | Seconds for the whole request                             |
| verify_tls       | input     | bool        | `true`               | Check the https certificate and host name                 |
| follow_redirects | input     | bool        | `false`              | Follow 3xx redirects (at most 10, http and https only)    |
| status_code      | output    | int         | `{status_code}`      | HTTP status code; `0` when there was no response          |
| response_body    | output    | std::string | `{response_body}`    | Response body                                             |
| response_headers | output    | std::string | `{response_headers}` | JSON object of response headers (lower-case names)        |
| error_message    | output    | std::string | `{error_message}`    | Why the request failed; empty on SUCCESS                  |

### Result

- A 2xx status is SUCCESS. Any other status is FAILURE, but `status_code`, `response_body` and
  `response_headers` are still written, so the tree can inspect them. To accept, say, a 404, wrap the
  node in `ForceSuccess` and check `status_code` afterwards.
- With no response (bad input, connection error, `timeout`, halt) the run is FAILURE, `status_code` is
  `0`, `response_body` is empty and `response_headers` is `{}`.
- Every run writes all four outputs, so a later node never reads a value left by an earlier run.
- `error_message` names the method and the URL up to `?`. The query string is left out, because it can
  hold a token.

### Request rules

- `query_parameters`, for example `{"id": "42", "tags": ["a", "b"], "verbose": null}`:
  - A string is used as it is; a number or boolean as its JSON text.
  - An array repeats the key (`tags=a&tags=b`). `null` sends the key alone (`verbose`).
  - Keys and values are percent-encoded, in the order written, and added after any query already in `url`.
- `headers`, for example `{"Authorization": "Bearer abc", "Accept": "application/json"}`:
  - Values are strings, numbers or booleans; `null` sends an empty header.
  - A name must be a valid header token, and a value may not hold a line break. This stops a port value
    from adding header lines of its own. `content_type` is checked the same way.
- Body:
  - GET never sends a body; a non-empty `body` fails.
  - POST, PUT and PATCH always send `body` (it may be empty), with `Content-Type: <content_type>`.
  - DELETE sends a body only when `body` is not empty.
  - A `Content-Type` in `headers` replaces `content_type`.
- JSON typed straight into a port works, for example `body='{"state": "idle"}'`. BT.CPP would normally
  read text in braces as a blackboard key; this Behavior takes it as text when it parses as JSON. A
  `{key}` reference is never valid JSON, so it is still read from the blackboard.

### Running and halting

- The request runs in the background, so the rest of the tree keeps ticking.
- A halt cancels the request within about 50 ms.
- Proxy settings come from the standard environment variables (`http_proxy`, `https_proxy`, `no_proxy`).

### Using status_code in a string port

`status_code` is an `int`, so Script expressions such as `status_code == 200` work. BT.CPP does not let a
string port share a blackboard key with an `int` output. To log the code or send it as text, make a text
copy first:

```xml
<Action ID="Script" code="report := 'Server answered HTTP ' .. status_code"/>
<Action ID="LogMessage" message="{report}"/>
```

## Replacing SendJsonHttp

`SendJsonHttp` was removed. `SendHttpRequest` sends the same request:

| SendJsonHttp port           | SendHttpRequest                                                         |
| --------------------------- | ----------------------------------------------------------------------- |
| `url`                       | `url`                                                                   |
| `method` (`POST` or `PUT`, default `POST`) | `method`. Set it: the default here is `GET`              |
| `payload` (default `{json}`) | `body="{json}"`; `content_type` already defaults to `application/json` |
| `timeout`                   | `timeout`                                                               |
| `response` (default `{response}`) | `response_body="{response}"`                                      |
| `status_code`               | `status_code`                                                           |
| (sent `Accept: application/json`) | `headers='{"Accept": "application/json"}'`                        |
| (refused invalid JSON before sending) | Put `CreateJson initial="{json}" json="{json}"` in front     |

Before and after:

```xml
<SendJsonHttp url="http://192.168.1.20:8080/status" payload="{msg}" response="{reply}"/>

<SendHttpRequest url="http://192.168.1.20:8080/status" method="POST" body="{msg}"
                 headers='{"Accept": "application/json"}' response_body="{reply}"/>
```

Differences that only add behavior: `SendHttpRequest` also writes `response_headers` and
`error_message`, writes its outputs when there is no response, and cancels faster on a halt.

## Security

- **HTTP sends in clear text.** Use `https://` when the request crosses a network you do not control.
- **Keep `verify_tls` true.** Set it to false only for a trusted device with a self-signed certificate on
  a network you control; it turns off both the certificate and the host-name check.
- **Keep secrets out of Objective files.** A token typed into `headers` is stored in the Objective XML.
  Prefer a blackboard value set at run time.
- **Treat the response as untrusted input.** Check every field you read (for example with
  `HasJsonField` and `GetJsonField`) before it drives motion.

## Example

Read job 42 from a job server, mark it done, and log the status code:

```xml
<Control ID="Sequence">
  <Action ID="SendHttpRequest" url="http://127.0.0.1:8080/api/jobs" method="GET"
          query_parameters='{"id": "42"}' headers='{"Accept": "application/json", "X-Api-Key": "my-key"}'
          timeout="5.0" status_code="{job_status}" response_body="{job}"/>
  <Action ID="SendHttpRequest" url="http://127.0.0.1:8080/api/jobs/42" method="PUT"
          body='{"state": "done"}' status_code="{update_status}"/>
  <Action ID="Script" code="report := 'Job server answered HTTP ' .. update_status"/>
  <Action ID="LogMessage" message="{report}"/>
</Control>
```

`GetJsonField` (JSON Behaviors) reads fields out of `{job}` and out of `{response_headers}`, for example
`path="/content-type"`.
