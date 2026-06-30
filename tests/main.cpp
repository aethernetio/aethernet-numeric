/*
 * Copyright 2026 Aethernet Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <unity.h>

void setUp() {};
void tearDown() {};

extern int test_tiered_int();
extern int test_tiered_int_view();
extern int test_fixed_point();
extern int test_exponential();
extern int test_exponential_arithmetic();
extern int test_exponential_math_policy();
extern int test_exponential_floating_runtime();
extern int test_text_io();
extern int test_ostream_io();
extern int test_wire_io();
extern int test_packed_ring();
extern int test_composed_types();
extern int test_composed_exponential();
extern int test_composed_exponential_tiered();
extern int test_fixed_math();
extern int test_type_size();

int main() {
  int res = 0;
  res += test_tiered_int();
  res += test_tiered_int_view();
  res += test_fixed_point();
  res += test_exponential();
  res += test_exponential_arithmetic();
  res += test_exponential_math_policy();
  res += test_exponential_floating_runtime();
  res += test_text_io();
  res += test_ostream_io();
  res += test_wire_io();
  res += test_packed_ring();
  res += test_composed_types();
  res += test_composed_exponential();
  res += test_composed_exponential_tiered();
  res += test_fixed_math();
  res += test_type_size();
  return res;
}
