#!/usr/bin/env bash
# 멈춰 ! — PC 회귀 테스트 실행기
set -e

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$TESTS_DIR/.build"
mkdir -p "$BUILD_DIR"

build_and_run() {
  local name="$1"
  local source="$2"
  shift 2

  g++ -std=c++11 -Wall -Wextra \
    -I "$TESTS_DIR/stubs" \
    "$@" \
    -o "$BUILD_DIR/$name" \
    "$source"

  "$BUILD_DIR/$name"
}

# 1) 두 번째 스트립 미연결(PIN_STRIP_2_VALUE=0) 구성 검증
build_and_run test_state_machine_no_strip2 "$TESTS_DIR/test_state_machine.cpp" \
  -DPIN_STRIP_2_VALUE=0

# 2) 두 번째 스트립을 D6에 연결한 현재 설정값 구성에서 #18의 독립 스트립 제어까지 검증
build_and_run test_state_machine_two_strips "$TESTS_DIR/test_state_machine.cpp" \
  -DPIN_STRIP_2_VALUE=6

# 3) 실물 축소판 D8 시작 / D9 끝 / D4 스트립
build_and_run test_reduced_sketch "$TESTS_DIR/test_reduced_sketch.cpp"

# 4) 주기 로그 비활성 설정에서도 상태 머신이 같게 동작하는지 검증
build_and_run test_state_machine_no_log "$TESTS_DIR/test_state_machine.cpp" \
  -DIR_LOG_INTERVAL_MS=0 -Wno-type-limits
