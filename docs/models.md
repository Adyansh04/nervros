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
  logged. A key that cannot be read leaves its provider's models out, and says why when one is
  asked, rather than stopping the app.
- Only free models run. On OpenRouter every id must end in `:free`, checked when the file loads
  and before every request, and `nervros-cli models --check` also checks the prices OpenRouter
  lists; `free_only = true` there says so, and `false` is refused. Elsewhere a free tier is a
  property of the key, which nothing here can check, so `free_only` is refused there: use a
  free-tier key.

## Models

```toml
[[model]]
id = "nemotron-super-or"                        # the name used in roles and the logs
provider = "openrouter"
model = "nvidia/nemotron-3-super-120b-a12b:free"  # the provider's own name
tools = true
limits = { rpm = 20, pool = "openrouter_free" }
privacy = { trains = false }
```

| Key | Default | |
|---|---|---|
| `vision` | `false` | Takes images. A turn with an image skips models without it. |
| `tools` | `false` | Calls tools. The agent needs it for anything but plain text. |
| `limits.rpm`, `limits.rpd` | none | Requests per minute and per day for this model. |
| `limits.pool` | none | A shared daily quota, from `[pools]`, such as OpenRouter's free requests across all its free models. |
| `privacy.local` | `false` | Runs on this machine; see the profile's `[privacy]`. |
| `privacy.trains` | `false` | The provider may train on what it is sent. |
| `params` | none | Request fields passed to the provider as they are, such as Gemini's `{ generation_config = { thinking_level = "low" } }`. |
| `context` | none | Its context window in tokens. Past half of it, the conversation is condensed before a turn: tool results before the operator's newest message are cut to a line, and when that is not enough a `summarise` model sums up the older part as goal, done, open and facts. Within a turn each request is cut to fit. The window shows how full it is; `/compact` condenses it at once, and right-clicking a message condenses up to it. Unset, nothing is cut. |
| `stream` | `false` | Its replies appear word by word in the window. |

```toml
[pools]
openrouter_free = { rpd = 50 }
```

## Roles

```toml
[roles]
routine = ["qwen3.5-9b-local"]        # conversation and tool calls
plan = ["gemini-3.8-flash"]           # advice when the routine model's plans keep failing
plan_check = ["gemini-3.8-flash"]     # a second opinion on each plan before it is shown
vision_check = ["qwen3.5-9b-local"]   # what `look` sees: questions about the camera frame
summarise = ["qwen3.5-9b-local"]      # captions and summaries
segment = ["gemini-3.5-flash-lite"]   # outlines for `segment`, points for `point`
```

A turn tries its role's models in order and skips a model that lacks what the turn needs, is over a
limit, or would break the privacy mode. Local models not in the list are tried after the others,
except for `segment` and `plan_check`: outlining is a skill few models have, and a second opinion
from the same model is no second opinion, so only the models listed are asked. `plan` is asked once
a request's plans have failed their checks twice, with the request, the skills, the last plan and
what was wrong with it; its few lines of advice go back to the routine model with the failure. It
is asked only when it lists other models than `routine`, and a model out of quota gives no advice.
Gemini's free tier outlines well:

```toml
[[provider]]
id = "gemini"
kind = "gemini_interactions"
key = { file = "~/.config/gemini.key" }

[[model]]
id = "gemini-3.5-flash-lite"
provider = "gemini"
model = "gemini-3.5-flash-lite"
vision = true
limits = { rpm = 10, rpd = 250 }
privacy = { trains = true }
```
A model that fails before the robot has acted is replaced by the next one; once the robot has
acted, the turn ends instead of repeating the action.

## Quotas

Every request is counted, per model and per pool, in `~/.local/state/nervros/quota.json`
(`$XDG_STATE_HOME/nervros` when set): a turn that calls tools makes several, and one the quota
refuses ends the turn there. Days roll over at midnight UTC, and for Gemini at midnight
Pacific time, as the providers count them. A model answering 429 is set aside for as long as its
`Retry-After` asks, a minute without one, and never past its provider's next midnight, when a
spent daily quota comes back. A model whose server does not answer, such as a local model that is
not running, is set aside for a minute, so the turns meanwhile go to the next model quietly. A
busy one (a 5xx, as Gemini's 503 under load) is asked once more two seconds later, unless part of
the turn already came back. A model that takes over from another gets the conversation without the
other's reasoning, which Gemma refuses with a 400, while a model carrying on gets its own back. The app's top bar shows the answering model with today's count, and
the dock's Models tab shows every role's chain and why a model is skipped.
