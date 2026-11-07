#!/usr/bin/env bash
#
# Runs the local test model for NervROS: Qwen3.5-9B (Q4_K_M) with its vision projector in
# llama.cpp's server, in a container, on an OpenAI-compatible API bound to localhost.
#
#   ./scripts/local-llm.sh start     # returns once the model answers
#   ./scripts/local-llm.sh status
#   ./scripts/local-llm.sh stop
#
# Fetch the model once, at the revision this script runs, with:
#   hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf mmproj-F16.gguf \
#       --revision 3885219b6810b007914f3a7950a8d1b469d598a5
set -euo pipefail

readonly NAME=nervros-llm
# Pinned by digest, because llama.cpp's flags change between builds (the same build canopy uses).
readonly IMAGE="${LLM_IMAGE:-ghcr.io/ggml-org/llama.cpp@sha256:014f721265464f38ccb247c1338d07d852c4bae7509a4b4734d07a2bbadc765c}"
readonly PORT="${LLM_PORT:-8081}"
readonly HF_CACHE="${HF_HOME:-${HOME}/.cache/huggingface}"
readonly REPO="${LLM_REPO:-models--unsloth--Qwen3.5-9B-GGUF}"
# Pinned too: a later download of a newer revision must not change the model that runs unseen.
readonly REVISION="${LLM_REVISION:-3885219b6810b007914f3a7950a8d1b469d598a5}"
readonly MODEL="${LLM_MODEL:-Qwen3.5-9B-Q4_K_M.gguf}"
readonly MMPROJ=mmproj-F16.gguf
readonly ALIAS="${LLM_ALIAS:-qwen3.5-9b}"
# 32k with an 8-bit KV cache fits the VRAM 16k took at 16 bits; the tools and persona alone are
# about 12k, so at 16k the conversation was summarised nearly every turn.
readonly CTX="${LLM_CTX:-32768}"
readonly KV="${LLM_KV:-q8_0}"

usage() {
    echo "usage: $0 start|stop|status"
}

running() { [[ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" == true ]]; }
healthy() { curl -sf "http://127.0.0.1:${PORT}/health" >/dev/null; }

start() {
    if running; then
        echo "${NAME} is already running"
        return
    fi
    # A stopped container left from an earlier run goes first; there may be none.
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    local -r files="${HF_CACHE}/hub/${REPO}/snapshots/${REVISION}"
    if [[ ! -e "${files}/${MODEL}" || ! -e "${files}/${MMPROJ}" ]]; then
        echo "${MODEL} or ${MMPROJ} at revision ${REVISION} is missing from ${HF_CACHE}; see the" \
            "header of this script" >&2
        exit 1
    fi
    local -r snapshot="/hf/hub/${REPO}/snapshots/${REVISION}"

    # --jinja so the chat template renders tools and tool calls; reasoning off because tool turns
    # parse more reliably without Qwen3.5's thinking block (llama.cpp issue 20837). One slot keeps
    # the whole context for a tool list plus an image, and the prompt cache stays on because every
    # turn shares the system prompt and tool schemas.
    docker run -d --name "${NAME}" --gpus all --memory 12g --cpus 8 \
        -p "127.0.0.1:${PORT}:8080" -v "${HF_CACHE}:/hf:ro" "${IMAGE}" \
        --model "${snapshot}/${MODEL}" --mmproj "${snapshot}/${MMPROJ}" --alias "${ALIAS}" \
        --ctx-size "${CTX}" --flash-attn on --cache-type-k "${KV}" --cache-type-v "${KV}" \
        --n-gpu-layers all --parallel 1 --jinja --reasoning off \
        --no-webui --host 0.0.0.0 --port 8080 >/dev/null

    local _
    for _ in $(seq 240); do
        if healthy; then
            echo "${NAME} serves ${ALIAS} at http://127.0.0.1:${PORT}/v1"
            return
        fi
        running || break
        sleep 1
    done
    docker logs --tail 30 "${NAME}" >&2
    echo "${NAME} did not come up" >&2
    exit 1
}

status() {
    if ! running; then
        echo "${NAME} is not running"
        return 1
    fi
    if healthy; then
        echo "${NAME} is up at http://127.0.0.1:${PORT}/v1"
    else
        echo "${NAME} is running but still loading"
    fi
    docker stats --no-stream --format '  {{.MemUsage}} RAM, {{.CPUPerc}} CPU' "${NAME}"
    # The VRAM line is extra: without nvidia-smi, or before the model is on the GPU, it is left out.
    nvidia-smi --query-compute-apps=process_name,used_memory --format=csv,noheader 2>/dev/null \
        | sed -n 's|.*llama-server, |  VRAM |p' || true
}

stop() {
    # Asked of docker first, so a docker that cannot answer fails here rather than reads as
    # "not running".
    local found
    found=$(docker ps -a --filter "name=^${NAME}\$" --format '{{.Names}}')
    if [[ -z "${found}" ]]; then
        echo "${NAME} was not running"
        return
    fi
    docker rm -f "${NAME}" >/dev/null
    echo "${NAME} stopped"
}

main() {
    case "${1:-}" in
        start) start ;;
        stop) stop ;;
        status) status ;;
        -h | --help) usage ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
}

main "$@"
