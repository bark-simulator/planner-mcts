load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file_content = """
cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    copts = ["-Iexternal/gtest/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)""",
    strip_prefix = "googletest-release-1.7.0",
)

http_archive(
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file = "@//tools/pybind11:pybind.BUILD",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
    remote = "https://github.com/patrickhart/rules_boost",
)



load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

# External dependency: Eigen; has no Bazel build.
http_archive(
    name = "com_github_eigen_eigen",
    build_file_content = """
cc_library(
    name = 'eigen',
    srcs = [],
    includes = ['.'],
    hdrs = glob(['Eigen/**']),
    visibility = ['//visibility:public'],
)""",
    sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = [
        "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
    ],
)

git_repository(
    name = "com_google_ceres_solver",
    commit = "e51e9b46f6ca88ab8b2266d0e362771db6d98067",
    remote = "https://github.com/ceres-solver/ceres-solver",
)

new_local_repository(
    name = "python_linux",
    path = "./python/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
    """
)

local_repository(
    name = "bark_project",
    #commit = "e927e967c2e97cb60c0c123b5030ecd4bc6db68c",
    #remote = "https://github.com/bark-simulator/bark"
    path="/home/bernhard/development/bark"
)

local_repository(
    name = "mamcts_project",
    #commit = "e927e967c2e97cb60c0c123b5030ecd4bc6db68c",
    #remote = "https://github.com/bark-simulator/bark"
    path="/home/bernhard/development/mamcts"
)
