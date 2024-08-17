// Copyright (c) 2024 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROMETHEUS_ROS__VISIBILITY_CONTROL_HPP_
#define PROMETHEUS_ROS__VISIBILITY_CONTROL_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VISIBILITY_EXPORT __attribute__((dllexport))
#define VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define VISIBILITY_EXPORT __declspec(dllexport)
#define VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef VISIBILITY_BUILDING_DLL
#define VISIBILITY_PUBLIC VISIBILITY_EXPORT
#else
#define VISIBILITY_PUBLIC VISIBILITY_IMPORT
#endif
#define VISIBILITY_PUBLIC_TYPE VISIBILITY_PUBLIC
#define VISIBILITY_LOCAL
#else
#define VISIBILITY_EXPORT __attribute__((visibility("default")))
#define VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define VISIBILITY_PUBLIC
#define VISIBILITY_LOCAL
#endif
#define VISIBILITY_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#endif  // #define PROMETHEUS_ROS__VISIBILITY_CONTROL_HPP_