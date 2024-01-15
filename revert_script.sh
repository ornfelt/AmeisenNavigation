#!/bin/bash

# Target commit to stop reverting
#TARGET_COMMIT="your_target_commit_id_here"
TARGET_COMMIT="f95e4760932b9dc07b9ab0ce9695ced65eec23d4"

# Current branch check
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT_BRANCH" != "master" ]; then
  echo "You are not on the master branch. Please switch to master before running this script."
  exit 1
fi

# Revert commits until the target commit is reached
#for COMMIT in $(git rev-list --reverse HEAD...$TARGET_COMMIT); do # Wrong (reversed)
for COMMIT in $(git rev-list HEAD ^$TARGET_COMMIT); do
    git revert --no-commit $COMMIT
	echo "Reverting $COMMIT"
    if [ $? -ne 0 ]; then
        echo "Conflict detected. Please resolve the conflict, and then run 'git revert --continue' to proceed."
        exit 1
    fi
done

# Commit the reverts
git commit -m "Reverted changes until $TARGET_COMMIT"

# End of script
echo "Reverts completed until commit $TARGET_COMMIT on branch $CURRENT_BRANCH."
# Now do: git push origin master