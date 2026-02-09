# Github Using Reminds

For a github new hand, the followings are commands I believe necessary for quick enters.

## Creating a new blah
### Creating a new repository

### Creating a new branch
```bash
git checkout -b your_branch_name
git add <your_file_name> # Or you may choose to use "git add ." to add all the files.
```

## Uploading files
### To main branch:
```bash
git add .
git commit -m "details_of_your_commit"
git push origin main
```
### To your own branch:
```bash
# The following two commands are not necessary, only to make sure about the branch.
git checkout your_branch_name # Before doing this, switch to your established branch.
git status # Make sure you are on the branch.

# The following commands are necessary.
git add <your_file_name>
git commit -m "details_of_your_commit"
git push -u origin your_branch_name
```

## Repository setting fatal
It is possible to encounter the situation below when attempting to git push a repository recently set up:
```bash
git push -u origin main
Username for 'https://github.com': your_user_name
Password for 'https://your_user_name@github.com': just_type_your_login_password
remote: Invalid username or token. Password authentication is not supported for Git operations.
fatal: Authentication failed for 'https://github.com/your_user_name/your_repository_name.git/'
```
> REASON: Github doesn't allow login password authentication, so this way of pushing is invalid.

Possible solution: using the SSH way to enter:
```bash
git remote set-url origin git@github.com:your_user_name/your_repository_name.git
```

The fatal may be encountered also in pushing a branch and so forth, while using the SSH way is usually effective.