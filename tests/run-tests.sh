#!/usr/bin/env bash
# ============================================================================
# 워크북 기준 상태 머신 회귀 테스트 실행기
#
# 실물 아두이노 없이 PC에서 `main.ino`의 상태 전이와 출력 논리를 검증합니다.
# 사용법: bash tests/run-tests.sh
# 필요 도구: g++ (C++11 이상)
# ============================================================================
set -e

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$TESTS_DIR/.build"
mkdir -p "$BUILD_DIR"

g++ -std=c++11 -Wall -Wextra \
  -I "$TESTS_DIR/stubs" \
  -o "$BUILD_DIR/test_state_machine" \
  "$TESTS_DIR/test_state_machine.cpp"

"$BUILD_DIR/test_state_machine"
