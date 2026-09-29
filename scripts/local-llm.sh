#!/usr/bin/env bash
#
# Runs the local test model for NervROS: Qwen3.5-9B (Q4_K_M) with its vision projector in
# llama.cpp's server, in a container, on an OpenAI-compatible API bound to localhost.
#
#   ./scripts/local-llm.sh start     # returns once the model answers
#   ./scripts/local-llm.sh status
#   ./scripts/local-llm.sh stop
#
# Fetch the model once with:
#   hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf mmproj-F16.gguf
set -euo pipefail

NAME=nervros-llm
# Pinned by digest, because llama.cpp's flags change between builds (the same build canopy uses).
IMAGE="${LLM_IMAGE:-ghcr.io/ggml-org/llama.cpp@sha256:014f721265464f38ccb247c1338d07d852c4bae7509a4b4734d07a2bbadc765c}"
PORT="${LLM_PORT:-8081}"
HF_CACHE="${HF_HOME:-${HOME}/.cache/huggingface}"
REPO="${LLM_REPO:-models--unsloth--Qwen3.5-9B-GGUF}"
MODEL="${LLM_MODEL:-Qwen3.5-9B-Q4_K_M.gguf}"
MMPROJ=mmproj-F16.gguf
ALIAS="${LLM_ALIAS:-qwen3.5-9b}"
CTX="${LLM_CTX:-16384}"

running() { [ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" = true ]; }
healthy() { curl -sf "http://127.0.0.1:${PORT}/health" >/dev/null; }

start() {
    if running; then
        echo "${NAME} is already running"
        return
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    model=$(compgen -G "${HF_CACHE}/hub/${REPO}/snapshots/*/${MODEL}" | head -n 1 || true)
    if [ -z "${model}" ] || [ ! -e "$(dirname "${model}")/${MMPROJ}" ]; then
        echo "${MODEL} or ${MMPROJ} is missing from ${HF_CACHE}; see the header of this script" >&2
        exit 1
    fi
    snapshot="/hf/$(dirname "${model#"${HF_CACHE}"/}")"

    # --jinja so the chat template renders tools and tool calls; reasoning off because tool turns
    # parse more reliably without Qwen3.5's thinking block (llama.cpp issue 20837). One slot keeps
    # the whole context for a tool list plus an image, and the prompt cache stays on because every
    # turn shares the system prompt and tool schemas.
    docker run -d --name "${NAME}" --gpus all --memory 12g --cpus 8 \
        -p "127.0.0.1:${PORT}:8080" -v "${HF_CACHE}:/hf:ro" "${IMAGE}" \
        --model "${snapshot}/${MODEL}" --mmproj "${snapshot}/${MMPROJ}" --alias "${ALIAS}" \
        --ctx-size "${CTX}" --n-gpu-layers all --parallel 1 --jinja --reasoning off \
        --no-webui --host 0.0.0.0 --port 8080 >/dev/null

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
    nvidia-smi --query-compute-apps=process_name,used_memory --format=csv,noheader 2>/dev/null \
        | sed -n 's|.*llama-server, |  VRAM |p'
}

case "${1:-}" in
    start) start ;;
    stop) docker rm -f "${NAME}" >/dev/null 2>&1 && echo "${NAME} stopped" || echo "${NAME} was not running" ;;
    status) status ;;
    *)
        echo "usage: $0 start|stop|status" >&2
        exit 2
        ;;
esac
