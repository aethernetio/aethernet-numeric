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

#ifndef AE_NUMERIC_TESTS_FOOTPRINT_AE_FP_COMMON_H_
#define AE_NUMERIC_TESTS_FOOTPRINT_AE_FP_COMMON_H_

#include <cstdint>

#if defined(_MSC_VER)
#define AE_FP_NOINLINE __declspec(noinline)
#else
#define AE_FP_NOINLINE __attribute__((noinline))
#endif

#if defined(_MSC_VER)
#define AE_FP_USED
#else
#define AE_FP_USED __attribute__((used))
#endif

#endif  // AE_NUMERIC_TESTS_FOOTPRINT_AE_FP_COMMON_H_
