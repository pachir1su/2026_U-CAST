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

# 1) main.ino 상태 머신·사이렌·로그
build_and_run test_state_machine "$TESTS_DIR/test_state_machine.cpp"

# 2) test/test.ino 축소판이 같은 사이렌·로그 동작을 하는지
build_and_run test_reduced_sketch "$TESTS_DIR/test_reduced_sketch.cpp"

# 3) 로그를 끈 설정(IR_LOG_INTERVAL_MS = 0)에서 주기 출력이 사라지는지
#    간격이 0이면 "간격이 지났는가" 비교가 항상 참이 되어 -Wtype-limits 경고가
#    납니다. 이 빌드에서만 나오는 경고이므로 여기서만 끕니다.
build_and_run test_state_machine_no_log "$TESTS_DIR/test_state_machine.cpp" \
  -DIR_LOG_INTERVAL_MS=0 -Wno-type-limits
