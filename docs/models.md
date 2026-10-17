# Models

Which models answer is configuration, never code. The models file, which the profile names, lists
providers (endpoints), the models on them, shared quotas and, for each role, the order to try
models in. [`profiles/example/models.toml`](../profiles/example/models.toml) has a local model and
a free OpenRouter one.

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
- An `openai_compat` provider needs `base_url`. Its `headers` go with each request, and
  `timeout_s` (60 by default) bounds each call.
- `key` is `{ file = "..." }` or `{ env = "NAME" }`, and absent for a local server. Keys are never
  logged. A key that cannot be read leaves its provider's models out, and says why when one of
  them is asked, rather than stopping the app.

Only free models run:

- On OpenRouter every id must end in `:free`, and `openrouter/...` and `stealth/...` ids are
  refused. This is checked when the file loads and before every request.
- `nervros-cli models --check` also checks the prices OpenRouter lists and reads its count of free
  requests today. `free_only = true` says so for OpenRouter, and `false` is refused.
- Elsewhere a free tier is a property of the key, which nothing here can check, so `free_only` is
  refused there: use a free-tier key.

## Models

```toml
[[model]]
id = "nemotron-super-or"                        # the name used in roles and the logs
provider = "openrouter"
model = "nvidia/nemotron-3-super-120b-a12b:free"  # the provider's own name
tools = true
limits = { rpm = 20, pool = "openrouter_free" }
privacy = { trains = true }                     # free endpoints may train on what they get
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
| `context` | none | Its context window in tokens; see below. Unset, nothing is cut. |
| `stream` | `false` | Its replies appear word by word in the window. |

With `context` set, the conversation is condensed before a turn once it passes half the window.
Tool results before the operator's newest message are cut to a line, and when that is not enough,
a `summarise` model sums up the older part as goal, done, open and facts. Within a turn, each
request is cut to fit. The window shows how full it is; `/compact` condenses it at once, and
right-clicking a message condenses everything up to it.

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
summarise = ["qwen3.5-9b-local"]      # condensing the conversation
segment = ["gemini-3.5-flash-lite"]   # outlines for `segment`, points for `point`
witness = ["er2-witness"]             # whether frames from before and after a mission show it done
moment = ["er2-moment"]               # which of a run of frames shows what the operator asks about
watch = ["gemma-4-26b"]               # whether a frame shows what a watch waits for, every 15 s
scene_check = ["er2-scene"]           # a plan the checks flagged, against what the cameras show
```

A turn tries its role's models in order. It skips a model that lacks what the turn needs, is over a
limit, or would break the privacy mode. `[roles]` must be there, even empty.

- Local models not in the list are tried after the others, except for `segment`, `plan_check` and
  `plan`. For those only the models listed are asked: outlining is a skill few models have, and a
  second opinion from the same model is no second opinion.
- `witness`, `moment` and `watch` left out use `vision_check`'s list. `plan_check` and
  `scene_check` are off while their lists are empty; `scene_check` sees camera frames and
  `plan_check` never does, so text in the scene cannot talk to the plan's critic.
- To share a scarce model's day between uses, give it one `[[model]]` id per use, each with its
  own `rpd`, and one pool for the provider's whole quota: on the G1, Gemini Robotics-ER 2's 20 a
  day are 10 for `witness`, 6 for `moment` and 4 for `scene_check`, and a use whose share is spent
  moves on to its next model. `nervros-cli models` shows what each has used today.
- `plan` is asked once a request's plans have failed their checks twice, with the request, the
  skills, the last plan and what was wrong with it. Its few lines of advice go back to the routine
  model with the failure. It is asked only when its list differs from `routine`'s, and a model out
  of quota gives no advice.
- A model that fails before the robot has acted is replaced by the next one. Once the robot has
  acted, the turn ends instead of repeating the action.

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

## Quotas

Every request is counted, per model and per pool, in `~/.local/state/nervros/quota.json`
(`$XDG_STATE_HOME/nervros` when set). A turn that calls tools makes several requests, and one the
quota refuses ends the turn there. Days roll over at midnight UTC, and for Gemini at midnight
Pacific time, as the providers count them.

- A model answering 429 is set aside for as long as its `Retry-After` asks, a minute without one,
  and never past its provider's next midnight, when a spent daily quota comes back.
- A model whose server does not answer, such as a local model that is not running, is tried last
  for a minute, by that process alone, so its turns meanwhile go quietly to the next model.
- A busy model (a 5xx, such as Gemini's 503 under load) is asked once more two seconds later,
  unless part of the turn already came back.
- When every model of a turn is held back and the first per-minute limit or 429 lifts within a
  minute, as after a burst of calls on free tiers, the turn says so and waits for it, once. Daily
  limits still end the turn at once.
- A model that takes over from another gets the conversation without the other's reasoning, which
  Gemma refuses with a 400. A model carrying on gets its own reasoning back.

The app's top bar shows the answering model with today's count. The Agent tab's Models list shows
every role's chain and why a model is skipped.
