# SJSU Robotics Control Systems

[Testing Document](https://docs.google.com/document/d/1GoI3ypr8xW_N2GrNbO3eNoYoz85498nix4rTv1WexoA/edit?usp=sharing)

## How to clone and create branch for submitting a pull request (PR).

1. Clone / Download the Repo  
   `git clone https://github.com/SJSURoboticsTeam/urc-control_systems-2021.git`
1. Change into the Repo's Directory  
   `cd urc-control_systems-2021/`
1. Create new branch from current branch (since this is new project it is master branch)  
   `git checkout -b some-branch-name`
1. Stage all the files you want to commit to git version control system. You can add multiple files at a time.  
   `git add name.cpp of.cpp files.cpp here.hpp`
1. Add a commit message describing what you changed.  
   `git commit -m 'example of a commit message'`
1. Push local git commits to github. Will push according to the branch you are currently on.  
   `git push`
1. Go to the Repo's GitHub website  
   `https://github.com/SJSURoboticsTeam/urc-control_systems-2021`
1. Go to the Pull Request tab and click 'New Pull Request' button.
   `https://github.com/SJSURoboticsTeam/urc-control_systems-2021/pulls`
1. Select your branch that you want to make a pull request with using the 'compare' drop down list.
1. Click 'Create Pull Request'
1. Within the pull request description make sure to link the Pull Request to a GitHub Issue found within the SJSURoboticsTeam/urc-central-2021 repo. Then add the following 'Resolves SJSURoboticsTeam/urc-central-2021/issues/#'  
   Example: `Resolves SJSURoboticsTeam/urc-central-2021/issues/100`

### Helpful Git Commands

`git status` - Shows all the files you have changed and have not yet committed.  
`git branch` - Shows the recent branches you've used and your current branch.  
`git pull origin/master` - Used for merging code with master when there are code conflicts. Need to be on development branch to use.
