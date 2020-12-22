#!/bin/bash

set +e

# We need to add a new remote for the upstream master, since this script could
# be running in a personal fork of the repository which has out of date branches.
git remote add upstream https://github.com/magazino/move_base_flex
git fetch upstream


newest_common_ancestor_sha=$(git merge-base ${GITHUB_HEAD_REF} upstream/${GITHUB_BASE_REF})
lines_diff_count=$(git clang-format --diff ${newest_common_ancestor_sha} | wc -l)

# in case of no diff we return a one-liner
if [ ${lines_diff_count} -gt 1 ]; then
	exit 1
else
	exit 0
fi
