# Models

Which models answer is configuration, never code. The models file, which the profile names, lists
providers (endpoints), models on them, shared quotas and, per role, the order to try models in.
[`profiles/example/models.toml`](../profiles/example/models.toml) has a local model and two free
OpenRouter ones.

## Providers

```toml
[[provider]]
id = "local"
kind = "openai_compat"
base_url = "http://127.0.0.1:8081/v1"
timeout_s = 120

[[provider]]
id = "openrouter"
kind = "openai_compat"
base_url = "https://openrouter.ai/api/v1"
key = { file = "~/.config/openrouter.key" }
free_only = true
headers = { "X-Title" = "NervROS" }
```

- `kind` is `openai_compat` for anything that speaks OpenAI's chat completions: llama.cpp, Ollama,
  vLLM, OpenRouter, Groq. `gemini_interactions` is Google's Gemini, whose key travels only in the
  `x-goog-api-key` header, never in a URL.
- `key` is `{ file = "..." }` or `{ env = "NAME" }`, and absent for a local server. Keys are never
  logged.
- `free_only = true` refuses at start-up any model on the provider that is not free. On OpenRouter
  that means ids ending in `:free`; `nervros-cli models --check` also checks the prices OpenRouter
  lists.

## Models

```toml
[[model]]
id = "qwen3.8-27b-or"            # the name used in roles and the logs
provider = "openrouter"
model = "qwen/qwen3.8-27b:free"  # the provider's own name
vision = true
tools = true
tool_choice = true
structured = "json_schema"
limits = { rpm = 20, pool = "openrouter_free" }
privacy = { trains = false }
```

| Key | Default | |
|---|---|---|
| `vision` | `false` | Takes images. A turn with an image skips models without it. |
| `tools` | `false` | Calls tools. The agent needs it for anything but plain text. |
| `tool_choice` | `false` | Honours a forced tool choice. |
| `structured` | `none` | `json_schema` when the model can be held to a JSON Schema. |
| `limits.rpm`, `limits.rpd` | none | Requests per minute and per day for this model. |
| `limits.pool` | none | A shared daily quota, from `[pools]`, such as OpenRouter's free requests across all its free models. |
| `privacy.local` | `false` | Runs on this machine; see the profile's `[privacy]`. |
| `privacy.trains` | `false` | The provider may train on what it is sent. |

```toml
[pools]
openrouter_free = { rpd = 50 }
```

## Roles

```toml
[roles]
routine = ["qwen3.5-9b-local"]        # conversation and tool calls
plan = ["qwen3.5-9b-local"]           # writing mission plans
vision_check = ["qwen3.5-9b-local"]   # what `look` sees: questions about the camera frame
summarise = ["qwen3.5-9b-local"]      # captions and summaries
```

A turn tries its role's models in order and skips a model that lacks what the turn needs, is over a
limit, or would break the privacy mode. Local models not in the list are tried after the others.
A model that fails before the robot has acted is replaced by the next one; once the robot has
acted, the turn ends instead of repeating the action.

## Quotas

Every request is counted, per model and per pool, in `~/.local/state/nervros/quota.json`
(`$XDG_STATE_HOME/nervros` when set): a turn that calls tools makes several, and one the quota
refuses ends the turn there. Days roll over at midnight UTC, and for Gemini at midnight
Pacific time, as the providers count them. A model answering 429 is set aside for a minute. The
app's top bar shows the answering model with today's count, and the dock's Models tab shows every
role's chain and why a model is skipped.
