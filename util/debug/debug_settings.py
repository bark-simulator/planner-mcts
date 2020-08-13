import sys
import os



cwd = os.getcwd()
workspace_folder = cwd
repo_paths = ["bark_project", "bark_project/python", "benchmark_database", "com_github_interaction_dataset_interaction_dataset", \
        "com_github_interaction_dataset_interaction_dataset/python", "planner_uct"]

executed_file = sys.argv[0]
tmp = "bark_hypothesis_uct/bazel-bin".join(executed_file.rsplit("bark_hypothesis_uct",1))
#tmp = tmp.replace("python_wrapper/", "bazel-bin/python_wrapper/")
runfiles_dir = tmp.replace(".py", ".runfiles")

sys.path.append(runfiles_dir)
for repo in repo_paths:
    full_path = os.path.join(runfiles_dir, repo)
    print("adding python path: {}".format(full_path))
    sys.path.append(full_path)

print(sys.path)