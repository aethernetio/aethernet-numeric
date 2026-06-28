/*
 * Copyright 2025 Aethernet Inc.
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
extern int test_exponential_floating_runtime();
extern int test_text_io();
extern int test_wire_io();
extern int test_type_size();

int main() {
  int res = 0;
  res += test_tiered_int();
  res += test_tiered_int_view();
  res += test_fixed_point();
  res += test_exponential();
  res += test_exponential_floating_runtime();
  res += test_text_io();
  res += test_wire_io();
  res += test_type_size();
  return res;
}
